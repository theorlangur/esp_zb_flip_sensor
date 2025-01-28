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
        GetAccelX,
        GetAccelY,
        GetAccelZ,
        GetTemp,
        GetGyroXRaw,
        GetGyroYRaw,
        GetGyroZRaw,
        GetGyroX,
        GetGyroY,
        GetGyroZ,
        GetPwrMgmt,
        SetPwrMgmt,
        GetPwrMgmt2,
        SetPwrMgmt2,
        SetAccelRange,
        SetGyroRange,
        ConfigFIFO,
        WriteMem,
        ReadMem,
        InitDMP,
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
        Internal              = 0,
        PLL_Gyro_X            = 1,
        PLL_Gyro_Y            = 2,
        PLL_Gyro_Z            = 3,
        PLL_Ext_Ref_32_768kHz = 4,
        PLL_Ext_Ref_19_2MHz   = 5,
        Reserved              = 6,
        Stop                  = 7
    };
    struct PwrMgmt
    {
        ClockSource clksel : 3 = ClockSource::Internal;
        uint8_t temp_dis   : 1 = 0;
        uint8_t unused1    : 1 = 0;
        uint8_t cycle      : 1 = 0;
        uint8_t sleep      : 1 = 0;
        uint8_t dev_reset  : 1 = 0;
    };

    enum class AccelFullScaleRange: uint8_t
    {
        _2_g  = 0,
        _4_g  = 1,
        _8_g  = 2,
        _16_g = 3,
    };
    struct AccelConfig
    {
        uint8_t unused            : 3 = 0;
        AccelFullScaleRange range : 2 = AccelFullScaleRange::_2_g;
        uint8_t x_self_test       : 1 = 0;
        uint8_t y_self_test       : 1 = 0;
        uint8_t z_self_test       : 1 = 0;
    };

    enum class GyroFullScaleRange: uint8_t
    {
        _250_deg_per_sec  = 0,
        _500_deg_per_sec  = 1,
        _1000_deg_per_sec = 2,
        _2000_deg_per_sec = 3,
    };
    struct GyroConfig
    {
        uint8_t unused            : 3 = 0;
        GyroFullScaleRange range  : 2 = GyroFullScaleRange::_250_deg_per_sec;
        uint8_t x_self_test       : 1 = 0;
        uint8_t y_self_test       : 1 = 0;
        uint8_t z_self_test       : 1 = 0;
    };
    struct FIFOEnabled
    {
        uint8_t slv0       : 1 = 0;
        uint8_t slv1       : 1 = 0;
        uint8_t slv2       : 1 = 0;
        uint8_t accel      : 1 = 0;
        uint8_t z_gyro     : 1 = 0;
        uint8_t y_gyro     : 1 = 0;
        uint8_t x_gyro     : 1 = 0;
        uint8_t temp       : 1 = 0;
    };

    enum class IntOpen: uint8_t { PushPull = 0, OpenDrain = 1 };
    enum class IntLevel: uint8_t { High = 0, Low = 1 };
    struct InterruptPinCfg
    {
        uint8_t  unused       : 1 = 0;
        uint8_t  i2c_bypass   : 1 = 0;
        uint8_t  fsync_en     : 1 = 0;
        uint8_t  fsync_level  : 1 = 0;
        uint8_t  int_rd_clear : 1 = 0;
        uint8_t  int_latch    : 1 = 0;
        IntOpen  int_open     : 1 = IntOpen::PushPull;
        IntLevel int_level    : 1 = IntLevel::High;
    };
    struct InterruptByte
    {
        uint8_t  data_rdy      : 1 = 0;
        uint8_t  dmp           : 1 = 0;
        uint8_t  pll_rdy       : 1 = 0;
        uint8_t  i2c_mst_int   : 1 = 0;
        uint8_t  fifo_overflow : 1 = 0;
        uint8_t  zero_motion   : 1 = 0;
        uint8_t  motion        : 1 = 0;
        uint8_t  free_fall     : 1 = 0;
    };
    struct DMPInterruptByte
    {
        uint8_t  dmp0      : 1 = 0;
        uint8_t  dmp1      : 1 = 0;
        uint8_t  dmp2      : 1 = 0;
        uint8_t  dmp3      : 1 = 0;
        uint8_t  dmp4      : 1 = 0;
        uint8_t  dmp5      : 1 = 0;
        uint8_t  unused    : 2 = 0;
    };
    struct MotionDetectionStatus
    {
        uint8_t  zero      : 1 = 0;
        uint8_t  unused    : 1 = 0;
        uint8_t  z         : 1 = 0;
        uint8_t  z_neg     : 1 = 0;
        uint8_t  y         : 1 = 0;
        uint8_t  y_neg     : 1 = 0;
        uint8_t  x         : 1 = 0;
        uint8_t  x_neg     : 1 = 0;
    };

    struct BankSel
    {
        uint8_t mem_sel       : 5 = 0;
        uint8_t cfg_user_bank : 1 = 0;
        uint8_t prefetch_en   : 1 = 0;
        uint8_t unused        : 1 = 0;
    };

    static ExpectedValue<MPU6050> Open(i2c::I2CBusMaster &bus);

    ExpectedValue<uint8_t> GetId();
    ExpectedValue<int16_t> GetAccelXRaw();
    ExpectedValue<int16_t> GetAccelYRaw();
    ExpectedValue<int16_t> GetAccelZRaw();
    ExpectedValue<int16_t> GetTempRaw();
    ExpectedValue<int16_t> GetGyroXRaw();
    ExpectedValue<int16_t> GetGyroYRaw();
    ExpectedValue<int16_t> GetGyroZRaw();
    ExpectedValue<float> GetAccelX();
    ExpectedValue<float> GetAccelY();
    ExpectedValue<float> GetAccelZ();
    ExpectedValue<float> GetGyroX();
    ExpectedValue<float> GetGyroY();
    ExpectedValue<float> GetGyroZ();

    ExpectedValue<PwrMgmt> GetPwrMgmt();
    ExpectedResult SetPwrMgmt(PwrMgmt v);

    ExpectedValue<PwrMgmt2> GetPwrMgmt2();
    ExpectedResult SetPwrMgmt2(PwrMgmt2 v);

    ExpectedResult SetAccelRange(AccelFullScaleRange v);
    ExpectedResult SetGyroRange(GyroFullScaleRange v);
    AccelFullScaleRange GetAccelRange() const;
    GyroFullScaleRange GetGyroRange() const;

    ExpectedResult ConfigFIFO(FIFOEnabled v);

    ExpectedResult Start();
    ExpectedResult Sleep();

    ExpectedResult InitDMP();
private:
    MPU6050(i2c::I2CDevice &&d);

    float GetAccelFromRaw(int16_t v) const;
    float GetGyroFromRaw(int16_t v) const;

    ExpectedResult WriteMem(uint16_t mem_addr, uint16_t length, uint8_t *data);
    ExpectedResult ReadMem(uint16_t mem_addr, uint16_t length, uint8_t *data);

    enum class Reg: uint8_t
    {
        GyroConfig            = 0x1B,
        AccelConfig           = 0x1C,
        FreeFallThreshold     = 0x1D,
        FreeFallDuration      = 0x1E,
        MotionThreshold       = 0x1F,
        MotionDuration        = 0x20,
        ZeroMotionThreshold   = 0x21,
        ZeroMotionDuration    = 0x22,
        FIFOEnabled           = 0x23,
        IntConfig             = 0x37,
        IntEnable             = 0x38,
        DMPIntStatus          = 0x39,
        IntStatus             = 0x3A,
        AccelXOut             = 0x3B,
        AccelYOut             = 0x3D,
        AccelZOut             = 0x3F,
        Temp                  = 0x41,
        GyroXOut              = 0x43,
        GyroYOut              = 0x45,
        GyroZOut              = 0x47,
        MotionDetectionStatus = 0x61,
        PwrMgmt               = 0x6B,
        PwrMgmt2              = 0x6C,
        BankSel               = 0x6D,
        MemStart              = 0x6E,
        MemRW                 = 0x6F,
        DMPCfg1               = 0x70,
        DMPCfg2               = 0x71,
        FIFOCount             = 0x72,
        FIFOData              = 0x74,
        WhoAmI                = 0x75,
    };

    using reg_who_am_i                = i2c::helpers::Register<uint8_t, Reg::WhoAmI, i2c::helpers::RegAccess::Read>;
    using reg_accel_x                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_y                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelYOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_z                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelZOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_temp                    = i2c::helpers::RegisterMultiByte<int16_t, Reg::Temp, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_gyro_x                  = i2c::helpers::RegisterMultiByte<int16_t, Reg::GyroXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_gyro_y                  = i2c::helpers::RegisterMultiByte<int16_t, Reg::GyroYOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_gyro_z                  = i2c::helpers::RegisterMultiByte<int16_t, Reg::GyroZOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_pwr_mgmt                = i2c::helpers::Register<PwrMgmt, Reg::PwrMgmt, i2c::helpers::RegAccess::RW>;
    using reg_pwr_mgmt2               = i2c::helpers::Register<PwrMgmt2, Reg::PwrMgmt2, i2c::helpers::RegAccess::RW>;
    using reg_gyro_cfg                = i2c::helpers::Register<GyroConfig, Reg::GyroConfig, i2c::helpers::RegAccess::RW>;
    using reg_accel_cfg               = i2c::helpers::Register<AccelConfig, Reg::AccelConfig, i2c::helpers::RegAccess::RW>;
    using reg_fifo_count              = i2c::helpers::RegisterMultiByte<uint16_t, Reg::FIFOCount, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_fifo_data               = i2c::helpers::Register<uint8_t, Reg::FIFOData, i2c::helpers::RegAccess::RW>;
    using reg_fifo_enable             = i2c::helpers::Register<FIFOEnabled, Reg::FIFOEnabled, i2c::helpers::RegAccess::RW>;

    using reg_int_config              = i2c::helpers::Register<InterruptPinCfg, Reg::IntConfig, i2c::helpers::RegAccess::RW>;
    using reg_int_enable              = i2c::helpers::Register<InterruptByte, Reg::IntEnable, i2c::helpers::RegAccess::RW>;
    using reg_int_status              = i2c::helpers::Register<InterruptByte, Reg::IntStatus, i2c::helpers::RegAccess::Read>;
    using reg_dmp_int_status          = i2c::helpers::Register<DMPInterruptByte, Reg::DMPIntStatus, i2c::helpers::RegAccess::Read>;

    using reg_motion_detect_status    = i2c::helpers::Register<MotionDetectionStatus, Reg::MotionDetectionStatus, i2c::helpers::RegAccess::Read>;

    using reg_free_fall_threshold     = i2c::helpers::Register<uint8_t, Reg::FreeFallThreshold, i2c::helpers::RegAccess::RW>;
    using reg_free_fall_duration      = i2c::helpers::Register<uint8_t, Reg::FreeFallDuration, i2c::helpers::RegAccess::RW>;
    using reg_motion_threshold        = i2c::helpers::Register<uint8_t, Reg::MotionThreshold, i2c::helpers::RegAccess::RW>;
    using reg_motion_duration         = i2c::helpers::Register<uint8_t, Reg::MotionDuration, i2c::helpers::RegAccess::RW>;
    using reg_zero_motion_threshold   = i2c::helpers::Register<uint8_t, Reg::ZeroMotionThreshold, i2c::helpers::RegAccess::RW>;
    using reg_zero_motion_duration    = i2c::helpers::Register<uint8_t, Reg::ZeroMotionDuration, i2c::helpers::RegAccess::RW>;

    using reg_bank_sel                = i2c::helpers::Register<BankSel, Reg::BankSel, i2c::helpers::RegAccess::RW>;
    using reg_mem_start               = i2c::helpers::Register<uint8_t, Reg::MemStart, i2c::helpers::RegAccess::RW>;
    using reg_mem_data                = i2c::helpers::Register<uint8_t, Reg::MemRW, i2c::helpers::RegAccess::RW>;

    using reg_dmp_cfg1                = i2c::helpers::Register<uint8_t, Reg::DMPCfg1, i2c::helpers::RegAccess::RW>;
    using reg_dmp_cfg2                = i2c::helpers::Register<uint8_t, Reg::DMPCfg2, i2c::helpers::RegAccess::RW>;

    using reg_dmp_addr                = i2c::helpers::RegisterMultiByte<uint16_t, Reg::BankSel, i2c::helpers::RegAccess::Write, i2c::helpers::ByteOrder::BE>;

    using reg_prog_addr               = i2c::helpers::RegisterMultiByte<uint16_t, Reg::DMPCfg1, i2c::helpers::RegAccess::RW, i2c::helpers::ByteOrder::BE>;

    i2c::I2CDevice m_Device;
    struct {
        AccelFullScaleRange accel_range : 2 = AccelFullScaleRange::_2_g;
        GyroFullScaleRange  gyro_range  : 2 = GyroFullScaleRange::_250_deg_per_sec;
        uint8_t             dmp_loaded  : 1 = 0;
        uint8_t             unused      : 3 = 0;
    } m_State;
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
