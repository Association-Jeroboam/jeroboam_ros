#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>

extern int canIFace;
constexpr int MAX_FRAME_SIZE = 8;

int send_can_frame(struct can_frame * frame);