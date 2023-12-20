#include "M8128_data.hpp"

TurnOnSensor::TurnOnSensor(): Node("SRI_node"),Sampling_Time(0){
    memset(&Receive_Data,0,sizeof(Receive_Data));
    this->declare_parameter("usart_port_name", "/dev/ttyUSB0");
    this->declare_parameter("serial_baud_rate", 115200);
    usart_port_name = this->get_parameter("usart_port_name").as_string();
    serial_baud_rate = this->get_parameter("serial_baud_rate").as_int();
    M8128_data_publisher = this->create_publisher<sri_interface::msg::SixAxisFTS>("/M3733C_Force_Sensor", 10);
    RCLCPP_INFO(this->get_logger(), "Data ready");

    try{
        M8128_Serial.setPort(usart_port_name);
        M8128_Serial.setBaudrate(serial_baud_rate);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
        M8128_Serial.setTimeout(_time);
        M8128_Serial.open();
    }
    catch(serial::IOException& e){
        RCLCPP_ERROR(this->get_logger(),"Force sensor cannot open serial port. Please check the serial port cable!");
    }
    if(M8128_Serial.isOpen()){
        RCLCPP_INFO(this->get_logger(), "Force sensor serial port opened");
        M8128_Serial.write(sample_freq);
        M8128_Serial.write(start_cmd);
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurnOnSensor::sensor_run, this));
}


TurnOnSensor::~TurnOnSensor(){
    M8128_Serial.write(end_cmd);
    RCLCPP_INFO(this->get_logger(), "TurnOnSensor deconstruction fun!");
}

void TurnOnSensor::M8128_data_publisher_fun(){
    sri_interface::msg::SixAxisFTS sensor_msgs;
    sensor_msgs.x_force = M8128_data.X_force;
    sensor_msgs.y_force = M8128_data.Y_force;
    sensor_msgs.z_force = M8128_data.Z_force;
    sensor_msgs.x_torque = M8128_data.X_torque;
    sensor_msgs.y_torque = M8128_data.Y_torque;
    sensor_msgs.z_torque = M8128_data.Z_torque;
    sensor_msgs.sampling_time = Sampling_Time;
    sensor_msgs.frame_number = M8128_data.frame_number;

    M8128_data_publisher->publish(sensor_msgs);
}

unsigned char TurnOnSensor::Check_Sum(){
    unsigned char check_sum = 0, k;
    unsigned char Count_Number = 24;
    for(k = 0; k < Count_Number; k++){
        check_sum = check_sum + Receive_Data.rx[k + 5];
    }
    RCLCPP_INFO(this->get_logger(), "check_sum = %d", check_sum);
    return check_sum;
}



/**************************************
Function: 从串口读取数据
读取6轴力传感器数据并解析（一个数据包有2+2+2+24+1=31个byte)
example: 55 00 1B D1 EB 41 A3 A4 3E CA D1 B4 3F 36 06 12 BE 6A 6F 83 3B 24 02 56 BB 00 EA 3C 37 8B AA 
帧头-1：0x55 
数量-2：0x00 0x1B(27-byte不含帧头和数量) 
数据包index-2: 0xD1 0xEB 
数据位-24：0x41 0xA3 0xA4 0x3E 0xCA 0xD1 0xB4 0x3F 0x36 0x06 0x12 0xBE 0x6A 0x6F 0x83 0x3B 0x24 0x02 0x56 0xBB 0x00 0xEA 0x3C 0x37 
sum校验(仅仅对数据位求和校验)-1： 0x8B
帧尾-1：0xAA 
***************************************/
bool TurnOnSensor::Get_Sensor_Data(){
    short j = 0, Header_Pos = 0, Tail_Pos = 0;  //中间变量
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0};
    unsigned char byte[4] = {0};
    M8128_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));   //读串口数据

    // step1: 找到帧头和尾
    for(j=0;j<31;j++){
        if(Receive_Data_Pr[j]==FRAME_HEADER)
            Header_Pos=j;
        else if(Receive_Data_Pr[j]==FRAME_TAIL)
            Tail_Pos=j;    
    }

    // step2: 根据不同情况写入数据帧原始数据
    // 情况1-帧头和尾的间隔30
    if(Tail_Pos==(Header_Pos+30)){
        //ROS_INFO("1----");
        memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    // 情况2-帧头和尾相邻
    else if(Header_Pos==(1+Tail_Pos)){
        //ROS_INFO("2----");
        for(j=0;j<31;j++)
          Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%31];
    }
    // 情况3-读出出错
    else {
        //ROS_INFO("3----");
        return false;
    }    

    // 取出数据帧原始数据的头和尾
    Receive_Data.Frame_Header = Receive_Data.rx[0]; //数据的第一位是帧头（固定值）
    Receive_Data.Frame_Tail = Receive_Data.rx[30];  //数据的最后一位是帧尾（数据校验位）

    // 判断数据可靠性
    
    if (Receive_Data.Frame_Header == FRAME_HEADER ){
        //判断帧头
        if (Receive_Data.Frame_Tail == FRAME_TAIL) {
            //判断帧尾 
            if (Receive_Data.rx[29] == Check_Sum()){
                //校验位检测

                // 数据无误可以保存
                M8128_data.frame_number =  Receive_Data.rx[3]*256 + Receive_Data.rx[4]; // frame number

                // 六轴数据
                byte[0] = Receive_Data.rx[5];
                byte[1] = Receive_Data.rx[6];
                byte[2] = Receive_Data.rx[7];
                byte[3] = Receive_Data.rx[8];
                Byte_to_Float(&(M8128_data.X_force), byte);

                byte[0] = Receive_Data.rx[9];
                byte[1] = Receive_Data.rx[10];
                byte[2] = Receive_Data.rx[11];
                byte[3] = Receive_Data.rx[12];
                Byte_to_Float(&(M8128_data.Y_force), byte);

                byte[0] = Receive_Data.rx[13];
                byte[1] = Receive_Data.rx[14];
                byte[2] = Receive_Data.rx[15];
                byte[3] = Receive_Data.rx[16];
                Byte_to_Float(&(M8128_data.Z_force), byte);

                byte[0] = Receive_Data.rx[17];
                byte[1] = Receive_Data.rx[18];
                byte[2] = Receive_Data.rx[19];
                byte[3] = Receive_Data.rx[20];
                Byte_to_Float(&(M8128_data.X_torque), byte);

                byte[0] = Receive_Data.rx[21];
                byte[1] = Receive_Data.rx[22];
                byte[2] = Receive_Data.rx[23];
                byte[3] = Receive_Data.rx[24];
                Byte_to_Float(&(M8128_data.Y_torque), byte);

                byte[0] = Receive_Data.rx[25];
                byte[1] = Receive_Data.rx[26];
                byte[2] = Receive_Data.rx[27];
                byte[3] = Receive_Data.rx[28];
                Byte_to_Float(&(M8128_data.Z_torque), byte);
                
                return true;
            }
        }
    } 
    return false;
}

void TurnOnSensor::sensor_run(){
    last_time = this->get_clock()->now();
    while(rclcpp::ok()){
        now_ = this->get_clock()->now();
        if(true==Get_Sensor_Data()){
            Sampling_Time = (now_-last_time).seconds();
            M8128_data_publisher_fun();
            last_time = now_;
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SRI Six Axis Force Torque Sensor Data");
    auto node = std::make_shared<TurnOnSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}