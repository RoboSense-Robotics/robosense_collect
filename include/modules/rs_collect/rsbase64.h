
#ifndef RSBASE64_H
#define RSBASE64_H

#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

class RSInterfaceBase64 {
 public:
  RSInterfaceBase64(){};
  // 注意: 如果不为virtual函数，会导致资源释放不完整的问题
  //      因为，不会通过多态性正确调用析构函数;
  virtual ~RSInterfaceBase64(){};

 public:
  // 设置版本
  virtual void m_setVersion(const int majorVersion, const int minorVersion) = 0;
  // 标记base64的版本
  virtual std::string m_version() = 0;
  // m_encode:
  // 输入: src / srcLen, 原始数据
  // 输出: dst / dstLen, base64编码数据
  //
  virtual int m_encode(unsigned char *src, const unsigned int srcLen,
                       unsigned char *dst, unsigned int &dstLen) = 0;
  // m_decode
  // 输入: dst / dstLen: base64编码数据
  // 输出: src / srcLen: 原始数据
  virtual int m_decode(unsigned char *dst, const unsigned int dstLen,
                       unsigned char *src, unsigned int &srcLen) = 0;
};

class RSBase64 : public RSInterfaceBase64 {
 public:
  RSBase64();
  ~RSBase64();

 public:
  virtual void m_setVersion(const int majorVersion, const int minorVersion);
  virtual std::string m_version();
  virtual int m_encode(unsigned char *src, const unsigned int srcLen,
                       unsigned char *dst, unsigned int &dstLen);
  virtual int m_decode(unsigned char *dst, const unsigned int dstLen,
                       unsigned char *src, unsigned int &srcLen);

 private:
  std::string m_base64Version;
};

class RSBase64Util {
 public:
  using Ptr = std::shared_ptr<RSBase64Util>;
  using ConstPtr = std::shared_ptr<const RSBase64Util>;

 public:
  RSBase64Util() {}
  ~RSBase64Util() {}

 public:
  static int encode(const std::string &src, std::string &dst) {
    dst.clear();
    const size_t org_size = src.size();
    std::vector<unsigned char> copy_src(org_size, '0');
    memcpy(copy_src.data(), src.data(), org_size);
    while (src.size() % 3 != 0) {
      copy_src.push_back(static_cast<unsigned char>('='));
    }

    std::vector<unsigned char> copy_dst(copy_src.size() / 3 * 4, '0');
    unsigned int dstLen = 0;
    int ret = RSBase64().m_encode(copy_src.data(), copy_src.size(),
                                  copy_dst.data(), dstLen);
    if (ret != 0) {
      return -1;
    }

    dst = std::string(reinterpret_cast<const char *>(copy_dst.data()), dstLen);

    return 0;
  }

  static int decode(const std::string &src, std::string &dst) {
    dst.clear();
    const size_t org_size = src.size();
    if (org_size % 4 != 0) {
      return -1;
    }

    std::vector<unsigned char> coyp_dst(org_size / 4 * 3, '0');
    unsigned int dstLen = 0;
    int ret = RSBase64().m_decode(
        const_cast<unsigned char *>(
            reinterpret_cast<const unsigned char *>(src.data())),
        org_size, coyp_dst.data(), dstLen);
    if (ret != 0) {
      return -2;
    }

    dst = std::string(reinterpret_cast<const char *>(coyp_dst.data()), dstLen);

    return 0;
  }
};

}  // namespace collect
}  // namespace rs_collect
}  // namespace robosense

#endif  // RSBASE64_H
