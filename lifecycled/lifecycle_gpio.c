#include "lifecycle_gpio.h"
#include "lc_log.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define DEVMEM_PAGE_SIZE 4096UL

/*
 * Direct C port of S65ar8030-transport-tx's `reset` action:
 *   devmem 0x11097020 32 0x00 ; sleep 1 ; devmem 0x11097020 32 0x08 ; sleep 1
 * `devmem` itself is just a thin CLI wrapper over exactly this mmap(/dev/mem)
 * sequence, so there's no behavior gap to preserve carefully here (unlike
 * pairing/persistence) -- just move it in.
 */
static int lc_gpio_reset_devmem(const lc_config_t* cfg)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        lc_log("lifecycle: open /dev/mem failed, errno=%d", errno);
        return -1;
    }

    unsigned long page_base = cfg->reset_devmem_addr & ~(DEVMEM_PAGE_SIZE - 1);
    unsigned long page_off  = cfg->reset_devmem_addr - page_base;

    void* map = mmap(NULL, DEVMEM_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)page_base);
    if (map == MAP_FAILED) {
        lc_log("lifecycle: mmap /dev/mem failed, errno=%d", errno);
        close(fd);
        return -1;
    }

    volatile unsigned int* reg = (volatile unsigned int*)((char*)map + page_off);

    *reg = 0x00;
    sleep(1);

    if (lifecycle_shutdown_requested()) {
        /* Finish the pulse we already started rather than leaving the
         * reset line asserted -- a chip left in reset until the next
         * daemon start is a worse outcome than a slightly-late shutdown. */
        lc_log("lifecycle: shutdown requested mid-reset, completing pulse before exit");
    }

    *reg = 0x08;
    sleep(1);

    munmap(map, DEVMEM_PAGE_SIZE);
    close(fd);
    return 0;
}

static int lc_gpio_write(int gpio, const char* attr, const char* value)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/%s", gpio, attr);

    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        lc_log("lifecycle: open %s failed, errno=%d", path, errno);
        return -1;
    }
    ssize_t wr = write(fd, value, strlen(value));
    close(fd);
    return (wr == (ssize_t)strlen(value)) ? 0 : -1;
}

int lc_gpio_read(int gpio, int* out)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return -1;
    }
    char buf[8] = {0};
    ssize_t rd  = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (rd <= 0) {
        return -1;
    }
    *out = (buf[0] == '1') ? 1 : 0;
    return 0;
}

static void lc_gpio_export(int gpio)
{
    char numbuf[16];
    snprintf(numbuf, sizeof(numbuf), "%d", gpio);

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        lc_log("lifecycle: open /sys/class/gpio/export failed, errno=%d", errno);
        return;
    }
    /* EBUSY/EEXIST here just means an earlier daemon run (or the shell
     * scripts this replaces) already exported it -- not a real failure,
     * so the return value is deliberately ignored. */
    (void)write(fd, numbuf, strlen(numbuf));
    close(fd);
}

/*
 * Direct C port of S97ar8030's `enable_rf()`. This is *not* a generic
 * "pulse N GPIOs the same way" loop -- the real sequence is asymmetric
 * (two pins raised together, then one, then the last; three of the four
 * revert to input at the end, one doesn't), so cfg->reset_gpios' array
 * *positions* carry meaning, not just their values:
 *   [0] = primary/enable pin (stays an output the whole time)
 *   [1],[2] = raised together first
 *   [3] = raised last
 * All four are pinned/exported/driven low first; [1],[2],[3] revert to
 * input at the end (matching enable_rf()'s own final `echo in > direction`
 * on gpio84/gpio83/gpio82, never gpio154). Pin *numbers* are configurable
 * (device-specific); this ordering and the delay sequence are not (a
 * property of the RF hardware bring-up spec itself, not a per-board knob
 * either script has ever needed to change).
 */
static int lc_gpio_reset_sysfs(const lc_config_t* cfg)
{
    if (cfg->reset_gpio_count != 4) {
        lc_log("lifecycle: gpio-sysfs reset needs exactly 4 GPIOs (primary, pair, pair, last), got %d", cfg->reset_gpio_count);
        return -1;
    }

    int primary = cfg->reset_gpios[0];
    int pair_a  = cfg->reset_gpios[1];
    int pair_b  = cfg->reset_gpios[2];
    int last    = cfg->reset_gpios[3];

    lc_gpio_export(primary);
    lc_gpio_write(primary, "direction", "out");
    lc_gpio_write(primary, "value", "0");

    lc_gpio_export(pair_a);
    lc_gpio_write(pair_a, "direction", "out");
    lc_gpio_write(pair_a, "value", "0");

    lc_gpio_export(pair_b);
    lc_gpio_write(pair_b, "direction", "out");
    lc_gpio_write(pair_b, "value", "0");

    lc_gpio_export(last);
    lc_gpio_write(last, "direction", "out");
    lc_gpio_write(last, "value", "0");

    usleep(100000);

    if (lifecycle_shutdown_requested()) {
        lc_log("lifecycle: shutdown requested mid-reset, completing pulse before exit");
    }

    lc_gpio_write(pair_a, "value", "1");
    lc_gpio_write(pair_b, "value", "1");

    usleep(50000);

    lc_gpio_write(primary, "value", "1");

    usleep(50000);

    lc_gpio_write(last, "value", "1");

    usleep(280000);

    lc_gpio_write(pair_b, "direction", "in");
    lc_gpio_write(pair_a, "direction", "in");
    lc_gpio_write(last, "direction", "in");

    return 0;
}

int lifecycle_gpio_reset(const lc_config_t* cfg)
{
    switch (cfg->reset_method) {
    case LC_RESET_METHOD_DEVMEM:
        return lc_gpio_reset_devmem(cfg);
    case LC_RESET_METHOD_GPIO_SYSFS:
        return lc_gpio_reset_sysfs(cfg);
    case LC_RESET_METHOD_NONE:
        lc_log("lifecycle: reset-method=none, not resetting the chip");
        return 0;
    default:
        lc_log("lifecycle: unknown reset method %d", (int)cfg->reset_method);
        return -1;
    }
}
