#include "Serial.h"
#define PI 3.1415926
SerialHandler::SerialHandler()
{
    portsign =-1;
    configStatu =-1;
    readByte = 0;
    readLen = 0;
    isReadOK = 0;
    Write_Priority = 0;
    DEV = "/dev/ttyUSB0";
    
    //needclean = false;
}
SerialHandler::~SerialHandler()
{
        Serial_Close();
}
void SerialHandler::init()
{
    if (portsign < 0)
         {
          portsign = open(DEV,O_RDWR | O_NOCTTY | O_NDELAY);
          // printf("portsign: %d\n", portsign);
          if (-1 ==portsign)
          { 
            cout<<"Open Port Fail"<<endl;
            
          }
          else
          {
            configStatu = ConfigPort(portsign); 
            if(configStatu<0) cout<<"Config Port Error!"<<endl;
            struct termios Opt;
            tcgetattr(portsign,&Opt);
            speed = cfgetospeed(&Opt);
          }
          unsigned char buff[1000];
          int cnt;
          cnt = read(portsign,buff,1000);
          for(int i=0;i<cnt;i++)
            cout<<(int)buff[i]<<" ";
          cout<<"init read"<<endl;
         }
}
void SerialHandler::Read()
{
    printf("queue size: %4d, ",fifo.size());
    if(Write_Priority!=0)
      {
        printf("writing now.\n");
        return;
      }
    if(FD_ISSET(portsign,&wd))
      {
        printf("write descriptor is set.\n");
      return;
    }
      
    // FD_ZERO(&rd);
    FD_SET(portsign,&rd);
    // if (FD_ISSET(portsign, &rd));
    // {
    //   printf("ready.\n");
    // }
    struct  timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    int temp = select(portsign+1, &rd, NULL, NULL, &tv);
    // cout << "temp" << temp << endl;
    // printf("is reading\n");
    if(/*select(portsign+1,&rd,NULL,NULL,NULL)*/temp <= 0)
    {
        // cout<<"Cannot select port!"<<endl;
	readLen = 0;
      // error or timeout. do nothing
    }
    else
    {
      // printf("read\n");
      unsigned char buff[100];
      readByte=read(portsign,buff,100);
      if(readByte<=0)
      {
      printf("no data.\n");
      return;
      }
      // readbuff[readByte]=0;
           
      for(int i=0;i<readByte;i++)
      {
        fifo.push(buff[i]);
      }
      if(fifo.size() > PKGSIZE*3)
      {
      cout<<fifo.size()<<" queue size>3*18, check loop rate"<<endl;
      }
         
      // put data into readbuff
      if(fifo.size()<PKGSIZE)
      {
        isReadOK = 0;
        //needclean = true;
        int ti=PKGSIZE;
        printf("%d < %d bytes in FIFO.\n",fifo.size(),ti);
      }
      else
      {
        uint8_t tsta = 0;
        queue<uint8_t> tmpfifo = fifo;
        uint8_t tcnt = 2;
        bool outflag = false;
        while(!tmpfifo.empty())
        {
          uint8_t ct = tmpfifo.front();
          tmpfifo.pop();
          switch(tsta)
          {
          case 0:
            if(ct == 0x55)
              tsta = 1;
            break;
          case 1:
            if(ct == 0xAA)
              tsta = 2;  //header finish
              if(tmpfifo.size()>=PKGSIZE-2)
              {
                // fifo remained size >= 16
                readbuff[0] = 0x55;
                readbuff[1] = 0xAA;
              }
              else
              {
                //needclean = true;
                return; // if not enough data in fifo, return
              }
            break;
          case 2:
              // enough data in fifo
              readbuff[tcnt]=ct;
              tcnt++;
              if(tcnt>=PKGSIZE)
              {
                isReadOK = 1;
                cout<<"readbuff: ";
                for(int i=0;i<PKGSIZE;i++)
                  cout<<(int)readbuff[i]<<" ";
                cout<<endl;
                if(tmpfifo.size()<PKGSIZE)
                {
                  fifo = tmpfifo;
                  outflag = true;   //whole package. and no more package in FIFO
                }
                else
                {
                  //get data again. another package remained
                  tcnt = 2;
                  tsta = 0;
                  outflag = false;
                }
              }   
            break;
          default:
              cerr<<"impossible!"<<endl;
            break;
          }
          if(outflag)
            break;
        }
      }
           
        //if (readLen == 0 && buff[0] != 0x55)
        //{
          //printf("error data\n");
          //return;
        //}
        //memcpy(&readbuff[readLen], buff, readByte);
        //readLen += readByte;
        //if (readLen == 14)
        //{
          //isReadOK = 1;
          //readLen = 0;
        //}
        //else if (readLen > 14)   // error
        //{
          //isReadOK = 0;
          //readLen = 0;
          //printf("read more than 14 bytes.\n");
        //}
           // printf("readByte: %d\n", readByte);
         // cout<<"readByte="<<readByte<<",readbuff:"<<readbuff[0]<<endl;

    }
    FD_ZERO(&rd);
}
//void SerialHandler::WritePose(double * res)
//{
   //ch[0] = 0xFA;
   //ch[1] = 0x0C;
   
   
   //for(int i=0;i<3;i++)
    //{  
    //double dOutput= res[i]  * 100;
    //int nOutput=int(dOutput);
    //uint8_t nOutHigh=nOutput>>8;
    //uint8_t nOutLow=nOutput & 0xff;
    
    //ch[2*i+2]=nOutHigh;
    //ch[2*i+3]=nOutLow;
    
   //}
    //for(int i=0;i<3;i++)
    //{  
    //double dOutput= res[i+3]  * 10000;
    //int nOutput=int(dOutput);
    //uint8_t nOutHigh=nOutput>>8;
    //uint8_t nOutLow=nOutput & 0xff;
    
    //ch[2*i+8]=nOutHigh;
    //ch[2*i+9]=nOutLow;
    
   //}
   
  
      
       //for(int i=0;i<14;i++)
       //printf("%X ",ch[i]);
       //cout<<endl;
    //Write_Priority = 1;
    //int flag=write(portsign,ch,14);
       //// cout<<flag<<endl; 
    //Write_Priority = 0;
//}

//void SerialHandler::WritePose(const ArucoLanding::marker_pose::ConstPtr& msg)
//{
     //ch[0] = 0xFA;
     //ch[1] = 0xFA;
     //ch[2] = 0x1A;
     //ch[3] = msg->id;
     //ch[4] = msg->state;
   
    //double res[6];
   //res[0] = msg->x;res[1] = msg->y; res[2] = msg->z;
   //res[3] = msg->roll;res[4] = msg->theta;res[5] = msg->yaw;
   //for(int i=0;i<3;i++)
    //{  
    //double dOutput= res[i]  * 10;
    //int nOutput=int(dOutput);
    //uint8_t nOutHigh=nOutput>>8;
    //uint8_t nOutLow=nOutput & 0xff;
    
    //ch[2*i+5]=nOutHigh;
    //ch[2*i+6]=nOutLow;
    
   //}
    //for(int i=0;i<3;i++)
    //{  
    //double dOutput= res[i+3]  * 10000;
    //int nOutput=int(dOutput);
    //uint8_t nOutHigh=nOutput>>8;
    //uint8_t nOutLow=nOutput & 0xff;
    
    //ch[2*i+11]=nOutHigh;
    //ch[2*i+12]=nOutLow;
    
   //}
      //ch[17] = msg->fitOK;
   //for(int i=0;i<5;i++)
    //{  
    //double dOutput= msg->t2Land[i]  * 100;
    //int nOutput=int(dOutput);
    //uint8_t nOutHigh=nOutput>>8;
    //uint8_t nOutLow=nOutput & 0xff;
    
    //ch[2*i+18]=nOutHigh;
    //ch[2*i+19]=nOutLow;
    
   //}

   //uint8_t xorsum = 0;
      //for(int i =5;i<28;i++)
      //{
       //xorsum ^= ch[i];
      //}

      //ch[28] = xorsum;

       //for(int i=0;i<29;i++)
       //printf("%X ",ch[i]);
       //cout<<endl;
    //Write_Priority = 1;
    //int flag=write(portsign,ch,29);
       //// cout<<flag<<endl; 
    //Write_Priority = 0;
//}
/*
void SerialHandler::WritePose(const PoseEstimate::marker_pose::ConstPtr& msg)
{
     ch[0] = 0xFA;
   
     ch[1] = 0x0D;
     ch[2] = msg->state;
   
    double res[6];
   res[0] = msg->x;res[1] = msg->y; res[2] = msg->z;
   res[3] = msg->roll;res[4] = msg->theta;res[5] = msg->yaw;
   for(int i=0;i<3;i++)
    {  
    double dOutput= res[i]  * 100;
    int nOutput=int(dOutput);
    uint8_t nOutHigh=nOutput>>8;
    uint8_t nOutLow=nOutput & 0xff;
    
    ch[2*i+3]=nOutHigh;
    ch[2*i+4]=nOutLow;
    
   }
    for(int i=0;i<3;i++)
    {  
    double dOutput= res[i+3]  * 10000;
    int nOutput=int(dOutput);
    uint8_t nOutHigh=nOutput>>8;
    uint8_t nOutLow=nOutput & 0xff;
    
    ch[2*i+9]=nOutHigh;
    ch[2*i+10]=nOutLow;
    
   }
   
  
      
       for(int i=0;i<15;i++)
       printf("%X ",ch[i]);
       cout<<endl;
    Write_Priority = 1;
    int flag=write(portsign,ch,15);
       // cout<<flag<<endl; 
    Write_Priority = 0;
}
*/
void SerialHandler::Serial_Close()
{
    close(portsign);
    portsign = -1;
}
int SerialHandler::ConfigPort(int fd)
{
    int status;
    struct termios opt;
    tcgetattr(fd,&opt);
    tcflush(fd,TCIOFLUSH);
    cfsetispeed(&opt,B115200);
    cfsetospeed(&opt,B115200);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8; //8位数据位
    opt.c_cflag &= ~PARENB;//无校验位
    opt.c_cflag &= ~CSTOPB;// 停止位
    opt.c_cc[VTIME] = 0; // 0.5 seconds
    opt.c_cc[VMIN] = 1;

    opt.c_cflag |= (CLOCAL | CREAD);

    opt.c_cflag &= ~HUPCL;
    opt.c_iflag &= ~INPCK;
    opt.c_iflag |= IGNBRK;
    opt.c_iflag &= ~ICRNL;
    opt.c_lflag &= ~IEXTEN;
    opt.c_lflag &= ~ECHOK;
    opt.c_lflag &= ~ECHOCTL;
    opt.c_lflag &= ~ECHOKE;
//    opt.c_cflag &= ~(IXON|IXOFF|IXANY);
//    opt.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
   opt.c_iflag &= ~(IXON |IXOFF|IXANY);
   opt.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);//原始输入模式
    opt.c_iflag &= ~(INLCR|IGNCR|ICRNL);
    opt.c_oflag &= ~(ONLCR|OCRNL);//屏蔽回车和换行
 //   opt.c_oflag &= ~OPOST;//原始输出模式
    status = tcsetattr(fd,TCSANOW,&opt);
    if (status !=0)
     {
      perror("tcsetattr fd error!");
      return -1;
     }
    tcflush( fd ,TCIOFLUSH );
    return 0;
}

//void SerialHandler::writeDestPos(const geometry_msgs::Pose::ConstPtr& msg)
//{
    //ch[0] = 0xFA;
    //ch[1] = 0xFA;
    //if (msg->orientation.w == 1.0)
    //{
      //ch[2] = 7;  // include xorsum!  current position
    //} else {
      //ch[2] = 8;  // move to next position
      //ch[10] = 1;
    //}

    //double res[3];
    //res[0] = msg->position.x;
    //res[1] = msg->position.y;
    //res[2] = msg->position.z;    
    //for (int i = 0; i < 3; i++)
    //{
        //int nOutput = int(res[i]*1000);   // 16 bits??
        //uint8_t nOutHigh = nOutput >> 8;
        //uint8_t nOutLow = nOutput & 0xFF;

        //ch[i*2+3] = nOutHigh;
        //ch[i*2+4] = nOutLow;
    //}

    //uint8_t xorsum = 0;
    //for (int i = 3; i < 9; i++)
    //{
        //xorsum ^= ch[i];
    //}
    //ch[9] = xorsum;

    //Write_Priority = 1;
    //if (ch[2] == 7)
    //{
      //printf("x: %f, y: %f, z: %f\n", res[0], res[1], res[2]);
      //write(portsign, ch, 10);
    //}
    //else
    //{
      //printf("------\n\nMove command: \nx: %f, y: %f, z: %f\n\n\n------\n", res[0], res[1], res[2]);
      //write(portsign, ch, 11);
    //}
    //Write_Priority = 0;

//}

void SerialHandler::writeCmd(int16_t PH,int16_t ZX,int16_t CS)
{
    ch[0] = 0x55;
    ch[1] = 0xAA;
    ch[2] = (PH>>8)&0xFF;
    ch[3] = PH&0xFF;
    ch[4] = (ZX>>8)&0xFF;
    ch[5] = ZX&0xFF;
    ch[6] = (CS>>8)&0xFF;
    ch[7] = CS&0xFF;
    
    Write_Priority = 1;
    int flag = write(portsign, ch, 8);
    Write_Priority = 0;

    //printf("Send cmd. %d\n", msg->data);
}


//void SerialHandler::writeMotionz(const geometry_msgs::Point32::ConstPtr& motionz_msg)
//{
    //cout << "send: " << motionz_msg->x << ", " << motionz_msg->y << "," << motionz_msg->z << endl;
    //ch[0] = 0xFA;
    //ch[1] = 0xFA;
    //ch[2] = 13;
    
    //uint32_t data = motionz_msg->x * 1000000;
    //ch[3] = (data & 0xFF000000) >> 24;
    //ch[4] = (data & 0x00FF0000) >> 16;
    //ch[5] = (data & 0x0000FF00) >> 8;
    //ch[6] = data & 0x000000FF;

    //data = motionz_msg->y * 1000000;
    //ch[7] = (data & 0xFF000000) >> 24;
    //ch[8] = (data & 0x00FF0000) >> 16;
    //ch[9] = (data & 0x0000FF00) >> 8;
    //ch[10] = data & 0x000000FF;

    //data = motionz_msg->z * 1000000;
    //ch[11] = (data & 0xFF000000) >> 24;
    //ch[12] = (data & 0x00FF0000) >> 16;
    //ch[13] = (data & 0x0000FF00) >> 8;
    //ch[14] = data & 0x000000FF;

    //uint8_t xorsum = 0;
    //for (int i = 3; i < 15; i++)
    //{
        //xorsum ^= ch[i];
    //}
    //ch[15] = xorsum;

    //// for (int i=0;i<15;i++)
    //// {
    ////   printf("%2x ",ch[i]);
    //// }
    //// cout<<endl;

    //Write_Priority = 1;
    //int flag = write(portsign, ch, 16);
    //Write_Priority = 0;
//}

//void SerialHandler::writeTestPos(const geometry_msgs::Pose msg)
//{
    //ch[0] = 0xFA;
    //ch[1] = 0xFA;
    //ch[2] = 7;  // include xorsum!

    //double res[3];
    //res[0] = msg.position.x;
    //res[1] = msg.position.y;
    //res[2] = msg.position.z;
    //printf("x: %f, y: %f, z: %f\n", res[0], res[1], res[2]);
    //for (int i = 0; i < 3; i++)
    //{
        //int nOutput = int(res[i]*1000);   // 16 bits??
        //uint8_t nOutHigh = nOutput >> 8;
        //uint8_t nOutLow = nOutput & 0xFF;

        //ch[i*2+3] = nOutHigh;
        //ch[i*2+4] = nOutLow;
    //}

    //uint8_t xorsum = 0;
    //for (int i = 3; i < 9; i++)
    //{
        //xorsum ^= ch[i];
    //}
    //ch[9] = xorsum;

    //Write_Priority = 1;
    //int flag = write(portsign, ch, 10);
    //Write_Priority = 0;

//}


