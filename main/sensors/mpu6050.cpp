#include "mpu6050.h"
#include <thread>

namespace dmp_internals
{
#include "mpu6050_dmp.ixx"
}

const char* MPU6050::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::Open: return "Open";
        case ErrorCode::GetId: return "GetId";
        case ErrorCode::GetAccelXRaw: return "GetAccelXRaw";
        case ErrorCode::GetAccelYRaw: return "GetAccelYRaw";
        case ErrorCode::GetAccelZRaw: return "GetAccelZRaw";
        case ErrorCode::GetAccelX: return "GetAccelX";
        case ErrorCode::GetAccelY: return "GetAccelY";
        case ErrorCode::GetAccelZ: return "GetAccelZ";
        case ErrorCode::GetTemp: return "GetAccelZRaw";
        case ErrorCode::GetGyroXRaw: return "GetGyroXRaw";
        case ErrorCode::GetGyroYRaw: return "GetGyroYRaw";
        case ErrorCode::GetGyroZRaw: return "GetGyroZRaw";
        case ErrorCode::GetGyroX: return "GetGyroX";
        case ErrorCode::GetGyroY: return "GetGyroY";
        case ErrorCode::GetGyroZ: return "GetGyroZ";
        case ErrorCode::GetPwrMgmt: return "GetPwrMgmt";
        case ErrorCode::SetPwrMgmt: return "SetPwrMgmt";
        case ErrorCode::GetPwrMgmt2: return "GetPwrMgmt2";
        case ErrorCode::SetPwrMgmt2: return "SetPwrMgmt2";
        case ErrorCode::SetAccelRange: return "SetAccelRange";
        case ErrorCode::SetGyroRange: return "SetGyroRange";
        case ErrorCode::ConfigFIFO: return "ConfigFIFO";
        case ErrorCode::WriteMem: return "WriteMem";
        case ErrorCode::ReadMem: return "ReadMem";
        case ErrorCode::InitDMP: return "InitDMP";
    }
}

MPU6050::ExpectedValue<MPU6050> MPU6050::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
    MPU6050 res(std::move(*r));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (auto xr = reg_gyro_cfg{res.m_Device}.Read(); !xr)
        return std::unexpected(Err{xr.error(), ErrorCode::Open});
    else
        res.m_State.gyro_range = xr.value().range;
    if (auto xr = reg_accel_cfg{res.m_Device}.Read(); !xr)
        return std::unexpected(Err{xr.error(), ErrorCode::Open});
    else
        res.m_State.accel_range = xr.value().range;

    return std::move(res);
}

MPU6050::MPU6050(i2c::I2CDevice &&d): m_Device(std::move(d))
{
}

MPU6050::ExpectedValue<uint8_t> MPU6050::GetId()
{
    return reg_who_am_i{m_Device}
            .Read()
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetId}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetAccelXRaw()
{
    int16_t v;
    return reg_accel_x{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelXRaw}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetAccelYRaw()
{
    int16_t v;
    return reg_accel_y{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelYRaw}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetAccelZRaw()
{
    int16_t v;
    return reg_accel_z{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelZRaw}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetTempRaw()
{
    int16_t v;
    return reg_temp{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetTemp}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetGyroXRaw()
{
    int16_t v;
    return reg_gyro_x{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroXRaw}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetGyroYRaw()
{
    int16_t v;
    return reg_gyro_y{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroYRaw}; });
}

MPU6050::ExpectedValue<int16_t> MPU6050::GetGyroZRaw()
{
    int16_t v;
    return reg_gyro_z{m_Device}
            .Read(v)
            .transform([&]{ return v; })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroZRaw}; });
}

float MPU6050::GetAccelFromRaw(int16_t v) const
{
    switch(m_State.accel_range)
    {
        case AccelFullScaleRange::_2_g:  return float(2) * v / std::numeric_limits<int16_t>::max();
        case AccelFullScaleRange::_4_g:  return float(4) * v / std::numeric_limits<int16_t>::max();
        case AccelFullScaleRange::_8_g:  return float(8) * v / std::numeric_limits<int16_t>::max();
        case AccelFullScaleRange::_16_g: return float(16) * v / std::numeric_limits<int16_t>::max();
    }
}

float MPU6050::GetGyroFromRaw(int16_t v) const
{
    switch(m_State.gyro_range)
    {
        case GyroFullScaleRange::_250_deg_per_sec:  return float(250) * v / std::numeric_limits<int16_t>::max();
        case GyroFullScaleRange::_500_deg_per_sec:  return float(500) * v / std::numeric_limits<int16_t>::max();
        case GyroFullScaleRange::_1000_deg_per_sec: return float(1000) * v / std::numeric_limits<int16_t>::max();
        case GyroFullScaleRange::_2000_deg_per_sec: return float(2000) * v / std::numeric_limits<int16_t>::max();
    }
}

MPU6050::ExpectedValue<float> MPU6050::GetAccelX()
{
    int16_t v;
    return reg_accel_x{m_Device}
            .Read(v)
            .transform([&]{  return GetAccelFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelX}; });
}

MPU6050::ExpectedValue<float> MPU6050::GetAccelY()
{
    int16_t v;
    return reg_accel_y{m_Device}
            .Read(v)
            .transform([&]{  return GetAccelFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelY}; });
}

MPU6050::ExpectedValue<float> MPU6050::GetAccelZ()
{
    int16_t v;
    return reg_accel_z{m_Device}
            .Read(v)
            .transform([&]{  return GetAccelFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetAccelZ}; });
}

MPU6050::ExpectedValue<float> MPU6050::GetGyroX()
{
    int16_t v;
    return reg_gyro_x{m_Device}
            .Read(v)
            .transform([&]{ return GetGyroFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroX}; });
}

MPU6050::ExpectedValue<float> MPU6050::GetGyroY()
{
    int16_t v;
    return reg_gyro_y{m_Device}
            .Read(v)
            .transform([&]{ return GetGyroFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroY}; });
}

MPU6050::ExpectedValue<float> MPU6050::GetGyroZ()
{
    int16_t v;
    return reg_gyro_z{m_Device}
            .Read(v)
            .transform([&]{ return GetGyroFromRaw(v); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetGyroZ}; });
}

MPU6050::ExpectedValue<MPU6050::PwrMgmt> MPU6050::GetPwrMgmt()
{
    return reg_pwr_mgmt{m_Device}
            .Read()
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetPwrMgmt}; });
}

MPU6050::ExpectedResult MPU6050::SetPwrMgmt(MPU6050::PwrMgmt v)
{
    return reg_pwr_mgmt{m_Device}
            .Write(v)
            .transform([&]{ return std::ref(*this); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::SetPwrMgmt}; });
}

MPU6050::ExpectedValue<MPU6050::PwrMgmt2> MPU6050::GetPwrMgmt2()
{
    return reg_pwr_mgmt2{m_Device}
            .Read()
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::GetPwrMgmt2}; });
}

MPU6050::ExpectedResult MPU6050::SetPwrMgmt2(MPU6050::PwrMgmt2 v)
{
    return reg_pwr_mgmt2{m_Device}
            .Write(v)
            .transform([&]{ return std::ref(*this); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::SetPwrMgmt2}; });
}

MPU6050::ExpectedResult MPU6050::SetAccelRange(AccelFullScaleRange v)
{
    return reg_accel_cfg{m_Device}
            .Write({.range = v})
            .transform([&]{ m_State.accel_range = v; return std::ref(*this); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::SetAccelRange}; });
}

MPU6050::ExpectedResult MPU6050::SetGyroRange(GyroFullScaleRange v)
{
    return reg_gyro_cfg{m_Device}
            .Write({.range = v})
            .transform([&]{ m_State.gyro_range = v; return std::ref(*this); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::SetGyroRange}; });
}

MPU6050::AccelFullScaleRange MPU6050::GetAccelRange() const
{
    return m_State.accel_range;
}

MPU6050::GyroFullScaleRange MPU6050::GetGyroRange() const
{
    return m_State.gyro_range;
}

MPU6050::ExpectedResult MPU6050::ConfigFIFO(FIFOEnabled v)
{
    return reg_fifo_enable{m_Device}
            .Write(v)
            .transform([&]{ return std::ref(*this); })
            .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::ConfigFIFO}; });
}

MPU6050::ExpectedResult MPU6050::Start()
{
    return SetPwrMgmt({.cycle = true, .sleep = false});
}

MPU6050::ExpectedResult MPU6050::Sleep()
{
    return SetPwrMgmt({.cycle = false, .sleep = true});
}

MPU6050::ExpectedResult MPU6050::WriteMem(uint16_t mem_addr, uint16_t length, uint8_t *data)
{
    return reg_dmp_addr{m_Device}
        .Write(mem_addr)
        .and_then([&]{ return m_Device.WriteRegMulti(uint8_t(Reg::MemRW), {data, length}, i2c::helpers::kTimeout); })
        .transform([&](i2c::I2CDevice &d){ return std::ref(*this); })
        .transform_error([&](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::WriteMem}; });
}

MPU6050::ExpectedResult MPU6050::ReadMem(uint16_t mem_addr, uint16_t length, uint8_t *data)
{
    return reg_dmp_addr{m_Device}
        .Write(mem_addr)
        .and_then([&]{ return m_Device.ReadRegMulti(uint8_t(Reg::MemRW), {data, length}, i2c::helpers::kTimeout); })
        .transform([&](i2c::I2CDevice &d){ return std::ref(*this); })
        .transform_error([&](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::ReadMem}; });
}

MPU6050::ExpectedResult MPU6050::InitDMP()
{
    constexpr uint16_t LOAD_CHUNK = 16;
    uint8_t cur[LOAD_CHUNK];
    if (m_State.dmp_loaded)
        return std::unexpected(Err{.i2cErr = {}, .code = ErrorCode::InitDMP});

    m_State.dmp_loaded = true;
    uint16_t this_write;
    for (uint16_t ii = 0; ii < dmp_internals::code_size; ii += this_write) {
        this_write = std::min(LOAD_CHUNK, uint16_t(dmp_internals::code_size - ii));
        //write
        if (auto r = WriteMem(ii, this_write, (uint8_t*)&dmp_internals::dmp_memory[ii]); !r)
            return std::unexpected(Err{.i2cErr = r.error().i2cErr, .code = ErrorCode::InitDMP});

        //verify
        if (auto r = ReadMem(ii, this_write, cur); !r)
            return std::unexpected(Err{.i2cErr = r.error().i2cErr, .code = ErrorCode::InitDMP});
        if (memcmp(dmp_internals::dmp_memory+ii, cur, this_write))
            return std::unexpected(Err{.i2cErr = {}, .code = ErrorCode::InitDMP});
    }

    return reg_prog_addr{m_Device}
                .Write(dmp_internals::sStartAddress)
                .transform([&]{ return std::ref(*this); })
                .transform_error([](::Err e){ return Err{.i2cErr = e, .code = ErrorCode::InitDMP}; });
}
