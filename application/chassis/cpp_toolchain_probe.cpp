// C++ 工具链探针 —— 仅用于验证嵌入式 C++ 编译/链接链路是否打通。
//
// 第 0 步（构建配置）落地后保留此文件作为冒烟测试：它行使后续 balance_state
// C++ 化将依赖的最小特性集（namespace / constexpr / if constexpr / extern "C"），
// 但不引入任何 STL 动态分配、异常、RTTI 或全局对象构造。
//
// 一旦真正的 C++ 内部模块就位，本文件可从 Makefile 的 CPP_SOURCES 中移除。

#include <cstdint>
#include <type_traits>

namespace balance::probe
{

// constexpr 编译期常量，验证 C++ 常量折叠按预期工作。
inline constexpr float kProbeScale = 2.0f;

// 模板 + if constexpr（C++17/20）：确认 -std=c++20 生效且无需 RTTI/异常。
template <typename T>
constexpr T ScaleValue(T value) noexcept
{
    if constexpr (std::is_floating_point_v<T>)
        return value * static_cast<T>(kProbeScale);
    else
        return value;
}

} // namespace balance::probe

// 以 C 链接暴露一个符号，验证 extern "C" 边界可被 C 侧（若需要）解析。
// 当前没有 C 调用方引用它，--gc-sections 会在链接时回收，不占最终体积。
extern "C" float BalanceCppToolchainProbe(float v)
{
    return balance::probe::ScaleValue(v);
}
