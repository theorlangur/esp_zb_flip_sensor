#ifndef MPU6050_H_
#define MPU6050_H_
#include "ph_i2c.hpp"

class MPU6050
{
public:
    static constexpr const uint8_t kAddress = 0b1101000;
    
    enum class ErrorCode: uint8_t
    {
        Ok,
        Open,
        GetId,
        GetAccelXRaw,
        GetAccelYRaw,
        GetAccelZRaw,
        GetPwrMgmt,
        SetPwrMgmt,
        GetPwrMgmt2,
        SetPwrMgmt2,
    };
    static const char* err_to_str(ErrorCode e);

    using Ref = std::reference_wrapper<MPU6050>;
    struct Err
    {
        ::Err i2cErr;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    template<class V>
    using ExpectedValue = std::expected<V, Err>;

    enum WakeUpFrequence: uint8_t
    {
        _1_25Hz = 0,
        _5_Hz   = 1,
        _20_Hz  = 2,
        _40_Hz  = 3,
    };

    struct PwrMgmt2
    {
        uint8_t stby_zg : 1 = 0;
        uint8_t stby_yg : 1 = 0;
        uint8_t stby_xg : 1 = 0;
        uint8_t stby_za : 1 = 0;
        uint8_t stby_ya : 1 = 0;
        uint8_t stby_xa : 1 = 0;
        WakeUpFrequence wake_ctrl : 2 = WakeUpFrequence::_1_25Hz;
    };

    enum class ClockSource: uint8_t
    {
        Internal = 0,
        PLL_Gyro_X = 1,
        PLL_Gyro_Y = 2,
        PLL_Gyro_Z = 3,
        PLL_Ext_Ref_32_768kHz = 4,
        PLL_Ext_Ref_19_2MHz = 5,
        Reserved = 6,
        Stop = 7
    };
    struct PwrMgmt
    {
        ClockSource clksel : 3 = ClockSource::Internal;
        uint8_t temp_dis : 1 = 0;
        uint8_t unused1  : 1 = 0;
        uint8_t cycle    : 1 = 0;
        uint8_t sleep    : 1 = 0;
        uint8_t dev_reset : 1 = 0;
    };

    static ExpectedValue<MPU6050> Open(i2c::I2CBusMaster &bus);

    ExpectedValue<uint8_t> GetId();
    ExpectedValue<int16_t> GetAccelXRaw();
    ExpectedValue<int16_t> GetAccelYRaw();
    ExpectedValue<int16_t> GetAccelZRaw();

    ExpectedValue<PwrMgmt> GetPwrMgmt();
    ExpectedResult SetPwrMgmt(PwrMgmt v);

    ExpectedValue<PwrMgmt2> GetPwrMgmt2();
    ExpectedResult SetPwrMgmt2(PwrMgmt2 v);
private:
    MPU6050(i2c::I2CDevice &&d);

    enum class Reg: uint8_t
    {
        AccelXOut = 0x3B,
        AccelYOut = 0x3D,
        AccelZOut = 0x3F,
        PwrMgmt = 0x6B,
        PwrMgmt2 = 0x6C,
        WhoAmI = 0x75,
    };

    using reg_who_am_i = i2c::helpers::Register<uint8_t, Reg::WhoAmI, i2c::helpers::RegAccess::Read>;
    using reg_accel_x  = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_y  = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelYOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_z  = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelZOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_pwr_mgmt = i2c::helpers::Register<PwrMgmt, Reg::PwrMgmt, i2c::helpers::RegAccess::RW>;
    using reg_pwr_mgmt2 = i2c::helpers::Register<PwrMgmt2, Reg::PwrMgmt2, i2c::helpers::RegAccess::RW>;

    i2c::I2CDevice m_Device;
};

template<>
struct tools::formatter_t<MPU6050::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::Err const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "{} at {}" , v.i2cErr, MPU6050::err_to_str(v.code));
    }
};

template<class V>
struct tools::formatter_t<MPU6050::ExpectedValue<V>>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::ExpectedValue<V> const& v)
    {
        if (v.has_value())
            return tools::format_to(std::forward<Dest>(dst), "{}" , v.value());
        else
            return tools::format_to(std::forward<Dest>(dst), "Err {}" , v.error());
    }
};

#endif
