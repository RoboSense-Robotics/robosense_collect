#include "modules/rs_collect/rsbase64.h"

namespace robosense {
namespace rs_collect {
namespace collect {

static char base64[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
                        'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
                        'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g',
                        'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r',
                        's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2',
                        '3', '4', '5', '6', '7', '8', '9', '+', '/'};

RSBase64::RSBase64() {
  char version[] = {'b', 'a', 's', 'e', '6', '4', '_',
                    'V', '_', '0', '_', '1', '\0'};
  m_base64Version = std::string(version);
}

RSBase64::~RSBase64() {
  // TODO...
}

void RSBase64::m_setVersion(const int majorVersion, const int minorVersion) {
  RS_UNUSED_PARAM(majorVersion);
  RS_UNUSED_PARAM(minorVersion);
}

std::string RSBase64::m_version() { return m_base64Version; }

int RSBase64::m_encode(unsigned char *src, const unsigned int srcLen,
                       unsigned char *dst, unsigned int &dstLen) {
  if (src == NULL || srcLen <= 0 || dst == NULL) {
    return -1;
  }

  int srcStep = 3;
  int srcOffset = 0;
  int dstStep = 4;
  int dstOffset = 0;
  while (srcOffset < srcLen) {
    unsigned int data = (((unsigned int)src[srcOffset]) << 16) |
                        (((unsigned int)src[srcOffset + 1]) << 8) |
                        (((unsigned int)src[srcOffset + 2]));

    dst[dstOffset] = base64[((data >> 18) & 0x3f)];
    dst[dstOffset + 1] = base64[((data >> 12) & 0x3f)];
    dst[dstOffset + 2] = base64[((data >> 6) & 0x3f)];
    dst[dstOffset + 3] = base64[data & 0x3f];

    dstOffset += dstStep;
    srcOffset += srcStep;
  }

  // 编码后的长度
  dstLen = dstOffset;

  return 0;
}

int RSBase64::m_decode(unsigned char *dst, const unsigned int dstLen,
                       unsigned char *src, unsigned int &srcLen) {
  if (src == NULL || dstLen <= 0 || dst == NULL) {
    return -1;
  }

  int srcStep = 3;
  int srcOffset = 0;
  int dstStep = 4;
  int dstOffset = 0;
  while (dstOffset < dstLen) {
    int iter;
    int mask[4] = {0};
    for (iter = 0; iter < sizeof(base64); ++iter) {
      if (dst[dstOffset + 0] == base64[iter] && mask[0] == 0) {
        dst[dstOffset + 0] = iter;
        mask[0] = 1;
      }

      if (dst[dstOffset + 1] == base64[iter] && mask[1] == 0) {
        dst[dstOffset + 1] = iter;
        mask[1] = 1;
      }

      if (dst[dstOffset + 2] == base64[iter] && mask[2] == 0) {
        dst[dstOffset + 2] = iter;
        mask[2] = 1;
      }

      if (dst[dstOffset + 3] == base64[iter] && mask[3] == 0) {
        dst[dstOffset + 3] = iter;
        mask[3] = 1;
      }
    }

    unsigned int data = (((unsigned int)(dst[dstOffset] & 0x3f) << 18)) |
                        (((unsigned int)(dst[dstOffset + 1] & 0x3f) << 12)) |
                        (((unsigned int)(dst[dstOffset + 2] & 0x3f) << 6)) |
                        (((unsigned int)(dst[dstOffset + 3] & 0x3f)));

    src[srcOffset] = ((data >> 16) & 0xff);
    src[srcOffset + 1] = ((data >> 8) & 0xff);
    src[srcOffset + 2] = (data & 0xff);

    srcOffset += srcStep;
    dstOffset += dstStep;
  }

  // 解码后的长度
  srcLen = srcOffset;

  return 0;
}

}  // namespace collect
}  // namespace rs_collect
}  // namespace robosense
