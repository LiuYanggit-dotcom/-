#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

using std::vector;

uint16_t calcCRC(const uint8_t *pBuffer, uint16_t bufferSize);
void decode_frame(vector<uint8_t> new_buffer, uint8_t *data_buffer);
void dump_imu570_data(uint8_t *data_buffer, float *imu_data);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_raw", 200);

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);

    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open imu: " << e.what());
        return -1;
    }

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open /dev/ttyUSB0");
        return -1;
    }

    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        try
        {
            size_t n = sp.available();
            if(n != 0)
            {
                uint8_t buffer[n];
                uint8_t data[36];
                float imu_data[9];

                n = sp.read(buffer, n);
                vector<uint8_t> vec_buffer(buffer, buffer + n);
                decode_frame(vec_buffer, data);
                dump_imu570_data(data, imu_data);

                sensor_msgs::Imu imu_raw;
                imu_raw.header.stamp = ros::Time::now();
                imu_raw.header.frame_id = "base_link";
                imu_raw.orientation = tf::createQuaternionMsgFromRollPitchYaw(imu_data[0], imu_data[1], imu_data[2]);
                imu_raw.angular_velocity.x = imu_data[3];
                imu_raw.angular_velocity.y = imu_data[4];
                imu_raw.angular_velocity.z = imu_data[5];
                imu_raw.linear_acceleration.x = imu_data[6];
                imu_raw.linear_acceleration.y = imu_data[7];
                imu_raw.linear_acceleration.z = imu_data[8];

                imu_pub.publish(imu_raw);
            }
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Serial read error: " << e.what());
            sp.close();
            try
            {
                sp.open();
            }
            catch(serial::IOException& e)
            {
                ROS_ERROR_STREAM("Unable to reopen imu: " << e.what());
                return -1;
            }
        }

        loop_rate.sleep();
    }

    sp.close();
    return 0;
}

void dump_imu570_data(uint8_t *recvbuf, float *imu_data)
{
    uint8_t k;
    unsigned char c_roll[4], c_pitch[4], c_yaw[4];
    unsigned char c_gx[4], c_gy[4], c_gz[4];
    unsigned char c_ax[4], c_ay[4], c_az[4];

    for (k = 0; k < 4; k++)
        c_roll[k] = recvbuf[k];
    imu_data[0] = *((float*)(c_roll));

    for (k = 0; k < 4; k++)
        c_pitch[k] = recvbuf[4 + k];
    imu_data[1] = *((float*)(c_pitch));

    for (k = 0; k < 4; k++)
        c_yaw[k] = recvbuf[8 + k];
    imu_data[2] = *((float*)(c_yaw));

    for (k = 0; k < 4; k++)
        c_gx[k] = recvbuf[12 + k];
    imu_data[3] = *((float*)(c_gx));

    for (k = 0; k < 4; k++)
        c_gy[k] = recvbuf[16 + k];
    imu_data[4] = *((float*)(c_gy));

    for (k = 0; k < 4; k++)
        c_gz[k] = recvbuf[20 + k];
    imu_data[5] = *((float*)(c_gz));

    for (k = 0; k < 4; k++)
        c_ax[k] = recvbuf[24 + k];
    imu_data[6] = *((float*)(c_ax));

    for (k = 0; k < 4; k++)
        c_ay[k] = recvbuf[28 + k];
    imu_data[7] = *((float*)(c_ay));

    for (k = 0; k < 4; k++)
        c_az[k] = recvbuf[32 + k];
    imu_data[8] = *((float*)(c_az));
}

uint16_t calcCRC(const uint8_t *pBuffer, uint16_t bufferSize)
{
    uint16_t poly = 0x8408;
    uint16_t crc = 0;
    uint8_t carry;
    uint8_t i_bits;
    uint16_t j;
    for (j = 0; j < bufferSize; j++)
    {
        crc = crc ^ pBuffer[j];
        for (i_bits = 0; i_bits < 8; i_bits++)
        {
            carry = crc & 1;
            crc = crc / 2;
            if (carry)
            {
                crc = crc ^ poly;
            }
        }
    }
    return crc;
}

void decode_frame(vector<uint8_t> new_buffer, uint8_t *data_buffer)
{
    static size_t protocol_len = 44;
    static vector<uint8_t> last_remaining_buffer;
    vector<uint8_t> cat_buffer;

    cat_buffer.insert(cat_buffer.end(), last_remaining_buffer.begin(), last_remaining_buffer.end());
    cat_buffer.insert(cat_buffer.end(), new_buffer.begin(), new_buffer.end());

    size_t data_len = cat_buffer.size();

    if(data_len < 5)
    {
        last_remaining_buffer.clear();
        last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin(), cat_buffer.end());
        return;
    }

    int i = 0;
    int j = 0;
    int index = 0;

    for(i; i < data_len-5; i++)
    {
        if(cat_buffer[i] == 0xFF && cat_buffer[i+1] == 0x02 && cat_buffer[i+2] == 0x90 && cat_buffer[i+3] == 0x00 && cat_buffer[i+4] == 0x24)
        {
            if(data_len - i + 1 >= protocol_len)
            {
                uint16_t poly = 0x8408;
                uint16_t crc = 0;
                uint8_t carry;
                uint8_t i_bits;
                uint16_t j;
                size_t bufferSize = 39;
                for (j = 0; j < bufferSize; j++)
                {
                    crc = crc ^ cat_buffer[i+2+j];
                    for (i_bits = 0; i_bits < 8; i_bits++)
                    {
                        carry = crc & 1;
                        crc = crc / 2;
                        if (carry)
                        {
                            crc = crc ^ poly;
                        }
                    }
                }
                uint16_t d = (cat_buffer[i+41] << 8) | cat_buffer[i+42];

                if(crc == d)
                {
                    for (int j = 0; j < 36; j++) {
                        data_buffer[j] = cat_buffer[i + j + 5];
                    }
                    last_remaining_buffer.clear();
                    last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin() + i + 44, cat_buffer.end());
                    return;
                }
            }
            else
            {
                last_remaining_buffer.clear();
                last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin(), cat_buffer.end());
                return;
            }
        }
    }

    if(i == data_len-5)
    {
        last_remaining_buffer.clear();
        last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer[i+1], cat_buffer[i+2]);
        return;
    }
}