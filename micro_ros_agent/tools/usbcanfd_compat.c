/* Keep an already configured Chuangxin USB device usable across SDK opens.
 *
 * Load this library with RTLD_LOCAL as the root of a dependency group whose
 * DT_NEEDED is the unmodified vendor libcontrolcanfd.so. Only that group's
 * usb_set_configuration calls are interposed; no LD_PRELOAD/global hook is used.
 * The vendor's duplicate SET_CONFIGURATION(1) leaves the tested V2.13 device
 * accepting bulk OUT but never completing bulk IN until USB power is cycled.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>

typedef struct usb_dev_handle usb_dev_handle;
extern int usb_control_msg(usb_dev_handle *, int, int, int, int, char *, int, int);

int usb_set_configuration(usb_dev_handle *device, int configuration)
{
    if (configuration > 0) {
        unsigned char active = 0;
        /* Standard GET_CONFIGURATION; no knowledge of libusb's handle layout. */
        int result = usb_control_msg(device, 0x80, 8, 0, 0,
                                     (char *)&active, 1, 1000);
        if (result < 0)
            return result;
        if (result != 1)
            return -EPROTO;
        if (active == configuration)
            return 0;
    }

    /* Genuine configuration changes and unconfiguration retain vendor behavior.
     * An unsuccessful GET above fails closed, rather than resetting blindly.
     */
    int (*original)(usb_dev_handle *, int) =
        dlsym(RTLD_NEXT, "usb_set_configuration");
    return original ? original(device, configuration) : -ENOSYS;
}
