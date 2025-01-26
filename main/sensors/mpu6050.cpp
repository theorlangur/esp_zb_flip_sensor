#include "mpu6050.h"
#include <thread>

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
        case ErrorCode::GetPwrMgmt: return "GetPwrMgmt";
        case ErrorCode::SetPwrMgmt: return "SetPwrMgmt";
        case ErrorCode::GetPwrMgmt2: return "GetPwrMgmt2";
        case ErrorCode::SetPwrMgmt2: return "SetPwrMgmt2";
    }
}

MPU6050::ExpectedValue<MPU6050> MPU6050::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
    MPU6050 res(std::move(*r));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
