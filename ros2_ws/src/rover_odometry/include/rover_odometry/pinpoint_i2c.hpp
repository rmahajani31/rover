#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rover_odometry
{
    class PinpointI2C
    {
        public:
            enum class Endian
            {
                Little,
                Big
            };
        
            PinpointI2C(int bus, int addr, Endian endian);
            ~PinpointI2C();

            PinpointI2C(const PinpointI2C &) = delete;
            PinpointI2C & operator=(const PinpointI2C &) = delete;

            PinpointI2C(PinpointI2C &&) = delete;
            PinpointI2C & operator=(PinpointI2C &&) = delete;

            std::uint32_t readU32(std::uint8_t reg);
            float readF32(std::uint8_t reg);

            void writeU32(std::uint8_t reg, std::uint32_t value);
            void writeF32(std::uint8_t reg, float value);

            void resetImu();
            void resetPosAndImu();
            void setEncoderDirections(bool x_reversed, bool y_reversed);
            void setTicksPerMm(float ticks_per_mm);
            void setPodOffsetsMm(float x_pod_offset_mm, float y_pod_offset_mm);
            void setYawScalar(float yaw_scalar);
            void setPositionMmRad(float x_mm, float y_mm, float yaw_rad);
        
        private:
            static constexpr std::uint32_t CTRL_RESET_IMU = 1u << 0;
            static constexpr std::uint32_t CTRL_RESET_IMU_AND_POS = 1u << 1;
            static constexpr std::uint32_t CTRL_SET_Y_REVERSED = 1u << 2;
            static constexpr std::uint32_t CTRL_SET_Y_FORWARD = 1u << 3;
            static constexpr std::uint32_t CTRL_SET_X_REVERSED = 1u << 4;
            static constexpr std::uint32_t CTRL_SET_X_FORWARD = 1u << 5;

            std::vector<std::uint8_t> readBytes(std::uint8_t reg, std::size_t length);
            void writeBytes(std::uint8_t reg, const std::vector<std::uint8_t> & data);
            void sendControl(std::uint32_t control_bits);

            std::uint32_t decodeU32(const std::vector<std::uint8_t> & bytes) const;
            float decodeF32(const std::vector<std::uint8_t> & bytes) const;
          
            std::vector<std::uint8_t> encodeU32(std::uint32_t value) const;
            std::vector<std::uint8_t> encodeF32(float value) const;

            std::string devicePath() const;

            int bus_;
            int addr_;
            Endian endian_;
    };

    PinpointI2C::Endian parseEndian(const std::string & endian_string);
}