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
        GetAllMeasurements,
        GetAllMeasurementsRaw,
        GetAccel,
        GetAccelRaw,
        GetAccelX,
        GetAccelY,
        GetAccelZ,
        GetTemp,
        GetGyro,
        GetGyroRaw,
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
        DMPSetOrientation,
        ConfigInterrupts,
        EnableInterrupts,
        GetEnableInterrupts,
        GetInterruptStatus,
        SignalPathReset,
        ConfigureMotionDetection,
        ConfigureZeroMotionDetection,
        SetMotionControl,
        GetMotionDetectionStatus,
        SetUserControl,
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

    struct orientation_t
    {
        enum class scalar_t: uint16_t{};
        friend uint16_t operator&(scalar_t s, int v) { return uint16_t(s) & v; }
        friend uint16_t operator>>(scalar_t s, int v) { return uint16_t(s) >> v; }
        friend uint16_t operator<<(scalar_t s, int v) { return uint16_t(s) << v; }
        friend uint16_t operator|(scalar_t s, int v) { return uint16_t(s) | v; }

        int8_t mtx[9] = {
        };

        constexpr uint16_t row_to_val(int row_idx) const
        {
            uint16_t res = 0;
            if (int8_t b = mtx[row_idx*3 + 0]; b != 0)
                res = 0 | ((b < 0) << 2);
            else if (int8_t b = mtx[row_idx*3 + 1]; b != 0)
                res = 1 | ((b < 0) << 2);
            else if (int8_t b = mtx[row_idx*3 + 2]; b != 0)
                res = 2 | ((b < 0) << 2);
            else
                res = 7;
            return res;
        }

        constexpr scalar_t to_scalar() const
        {
            uint16_t res = 0;
            res = row_to_val(0);
            res |= row_to_val(1) << 3;
            res |= row_to_val(2) << 6;
            return scalar_t(res);
        }
    };

    enum class DMPFeature: uint16_t
    {
        Tap = 0x001,
        AndroidOrient = 0x002,
        LowPowerQuaternions = 0x004,
        Pedometer = 0x008,
        LowPower6XQuaternions = 0x010,
        CalibratedGyro = 0x020,
        SendRawAccel = 0x040,
        SendRawGyro = 0x080,
        SendCalibratedGyro = 0x100
    };
    friend uint16_t operator|(DMPFeature f1, DMPFeature f2) { return uint16_t(f1) | uint16_t(f2); }
    friend uint16_t operator|(uint16_t f1, DMPFeature f2)   { return uint16_t(f1) | uint16_t(f2); }
    friend uint16_t operator|(DMPFeature f1, uint16_t f2)   { return uint16_t(f1) | uint16_t(f2); }

    enum class DMPInterrupt: uint8_t
    {
        Gesture = 0x01,
        Continuous = 0x02
    };

    enum class Orientation: uint8_t
    {
        Portrait         = 0x00,
        Landscape        = 0x01,
        ReversePortrait  = 0x02,
        ReverseLandscape = 0x03,
    };

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

    enum DecrementRate: uint8_t
    {
        Reset = 0,
        _1 = 1,
        _2 = 2,
        _4 = 3
    };
    struct MotionDetectionControl
    {
        DecrementRate motion_counter_decrement_rate    : 2 = DecrementRate::Reset;
        DecrementRate free_fall_counter_decrement_rate : 2 = DecrementRate::Reset;
        uint8_t accel_on_delay                         : 2 = 0;//ms
        uint8_t unused                                 : 2 = 0;
    };

    struct UserControl
    {
        uint8_t signal_path_total_reset : 1 = 0;
        uint8_t i2c_master_reset        : 1 = 0;
        uint8_t fifo_reset              : 1 = 0;
        uint8_t dmp_reset               : 1 = 0;
        uint8_t always_zero             : 1 = 0;
        uint8_t i2c_master_enable       : 1 = 0;
        uint8_t fifo_enable             : 1 = 0;
        uint8_t dmp_enable              : 1 = 0;
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

    struct AccelRaw
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct Accel
    {
        float x;
        float y;
        float z;
    };
    ExpectedValue<AccelRaw> GetAccelRaw();
    ExpectedValue<Accel> GetAccel();

    struct GyroRaw
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct Gyro
    {
        float x;
        float y;
        float z;
    };
    ExpectedValue<GyroRaw> GetGyroRaw();
    ExpectedValue<Gyro> GetGyro();

    struct AllRaw
    {
        AccelRaw accel;
        int16_t temp;
        GyroRaw gyro;
    };
    struct AllMeasurements
    {
        Accel accel;
        float temp;
        Gyro gyro;
    };
    ExpectedValue<AllRaw> GetAllRaw();
    ExpectedValue<AllMeasurements> GetAllMeasurements();

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

    ExpectedResult DMPSetOrientation(orientation_t::scalar_t orient);

    ExpectedResult ConfigureInterrupts(InterruptPinCfg cfg);

    ExpectedValue<InterruptByte> GetEnabledInterrupts();
    ExpectedResult EnableInterrupts(InterruptByte b);

    ExpectedValue<InterruptByte> GetInterruptStatus();

    struct signal_path_t
    {
        uint8_t temp   : 1 = 1;
        uint8_t accel  : 1 = 1;
        uint8_t gyro   : 1 = 1;
        uint8_t unused : 5 = 0;
    };
    ExpectedResult SignalPathReset(signal_path_t p);

    ExpectedResult ConfigureMotionDetection(uint8_t threshold, uint8_t duration, MotionDetectionControl ctrl);
    ExpectedResult ConfigureZeroMotionDetection(uint8_t threshold, uint8_t duration = 0);

    ExpectedValue<MotionDetectionStatus> GetMotionDetectionStatus();
    ExpectedResult SetUserCtrl(UserControl v);
private:
    MPU6050(i2c::I2CDevice &&d);

    float GetAccelFromRaw(int16_t v) const;
    float GetGyroFromRaw(int16_t v) const;
    float GetTempFromRaw(int16_t v) const;

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
        SignalPathReset       = 0x68,
        MotionDetectionControl= 0x69,
        UserCtrl              = 0x6A,
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
    using reg_signal_path_reset       = i2c::helpers::Register<signal_path_t, Reg::SignalPathReset, i2c::helpers::RegAccess::Write>;
    using reg_all_measurements        = i2c::helpers::RegisterMultiByte<AllRaw, Reg::AccelXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE, sizeof(int16_t)>;
    using reg_accel                   = i2c::helpers::RegisterMultiByte<AccelRaw, Reg::AccelXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE, sizeof(int16_t)>;
    using reg_accel_x                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_y                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelYOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_accel_z                 = i2c::helpers::RegisterMultiByte<int16_t, Reg::AccelZOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_temp                    = i2c::helpers::RegisterMultiByte<int16_t, Reg::Temp, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE>;
    using reg_gyro                    = i2c::helpers::RegisterMultiByte<GyroRaw, Reg::GyroXOut, i2c::helpers::RegAccess::Read, i2c::helpers::ByteOrder::BE, sizeof(int16_t)>;
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
    using reg_user_ctrl               = i2c::helpers::Register<UserControl, Reg::UserCtrl, i2c::helpers::RegAccess::RW>;

    using reg_int_config              = i2c::helpers::Register<InterruptPinCfg, Reg::IntConfig, i2c::helpers::RegAccess::RW>;
    using reg_int_enable              = i2c::helpers::Register<InterruptByte, Reg::IntEnable, i2c::helpers::RegAccess::RW>;
    using reg_int_status              = i2c::helpers::Register<InterruptByte, Reg::IntStatus, i2c::helpers::RegAccess::Read>;
    using reg_dmp_int_status          = i2c::helpers::Register<DMPInterruptByte, Reg::DMPIntStatus, i2c::helpers::RegAccess::Read>;

    using reg_motion_detect_status    = i2c::helpers::Register<MotionDetectionStatus, Reg::MotionDetectionStatus, i2c::helpers::RegAccess::Read>;
    using reg_motion_detect_control   = i2c::helpers::Register<MotionDetectionControl, Reg::MotionDetectionControl, i2c::helpers::RegAccess::RW>;

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

template<>
struct tools::formatter_t<MPU6050::Accel>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::Accel const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "Accel[X:{}; Y:{}; Z:{}]" , v.x, v.y, v.z);
    }
};

template<>
struct tools::formatter_t<MPU6050::AccelRaw>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::AccelRaw const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "AccelR[X:{}; Y:{}; Z:{}]" , v.x, v.y, v.z);
    }
};

template<>
struct tools::formatter_t<MPU6050::Gyro>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::Gyro const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "Gyro[X:{}; Y:{}; Z:{}]" , v.x, v.y, v.z);
    }
};

template<>
struct tools::formatter_t<MPU6050::GyroRaw>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::GyroRaw const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "GyroR[X:{}; Y:{}; Z:{}]" , v.x, v.y, v.z);
    }
};

template<>
struct tools::formatter_t<MPU6050::AllMeasurements>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::AllMeasurements const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "All: {}; {}; Temp: {}C" , v.accel, v.gyro, v.temp);
    }
};

template<>
struct tools::formatter_t<MPU6050::AllRaw>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::AllRaw const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "AllR: {}; {}; TempR: {}" , v.accel, v.gyro, v.temp);
    }
};

template<>
struct tools::formatter_t<MPU6050::MotionDetectionStatus>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::MotionDetectionStatus const& v)
    {
        const uint8_t *pS = (const uint8_t *)&v;
        return tools::format_to(std::forward<Dest>(dst), "{:x}" , *pS);
    }
};

template<>
struct tools::formatter_t<MPU6050::InterruptByte>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, MPU6050::InterruptByte const& v)
    {
        const uint8_t *pS = (const uint8_t *)&v;
        if (*pS == 0)
            return tools::format_to(std::forward<Dest>(dst), "[Int: empty]");
        using res_t = std::expected<size_t, FormatError>;
        size_t res = 0;
        return tools::format_to(std::forward<Dest>(dst), "[Int:")
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.dmp)
                    return tools::format_to(std::forward<Dest>(dst), " DMP:{}", uint8_t(v.dmp));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.motion)
                    return tools::format_to(std::forward<Dest>(dst), " M:{}", uint8_t(v.motion));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.pll_rdy)
                    return tools::format_to(std::forward<Dest>(dst), " PLL:{}", uint8_t(v.pll_rdy));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.data_rdy)
                    return tools::format_to(std::forward<Dest>(dst), " D:{}", uint8_t(v.data_rdy));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.free_fall)
                    return tools::format_to(std::forward<Dest>(dst), " FF:{}", uint8_t(v.free_fall));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.i2c_mst_int)
                    return tools::format_to(std::forward<Dest>(dst), " I2C_M:{}", uint8_t(v.i2c_mst_int));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.zero_motion)
                    return tools::format_to(std::forward<Dest>(dst), " ZM:{}", uint8_t(v.zero_motion));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                if (v.fifo_overflow)
                    return tools::format_to(std::forward<Dest>(dst), " FIFO_O:{}", uint8_t(v.fifo_overflow));
                else
                    return 0;
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                return tools::format_to(std::forward<Dest>(dst), "]");
            })
            .and_then([&](size_t s)->res_t
            { 
                res += s;
                return res;
            });
    }
};

#endif
