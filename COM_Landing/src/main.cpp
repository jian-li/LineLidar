#include "ros/ros.h"
#include <stdio.h>
#include "Serial.h"
#include <iostream>
#include <fstream>
#include "COM_Landing/cleanBot_msg.h"

using namespace std;
SerialHandler m_serial;

// for keyboard input
class kbdata
{
    public:
        kbdata();
        ~kbdata() { tcsetattr(0, TCSANOW, &original_terminal_settings_); };
        char getUserInput();

    private:

        struct termios original_terminal_settings_;
        struct termios new_terminal_settings_;
        struct timeval tv_;
        fd_set fds_;
};

kbdata::kbdata()
{
    // terminal settings, for user input. Refer to rpg_vikit/user_input_thread.cpp
    tcgetattr(0, &original_terminal_settings_); // save old terminal i/o settings
    new_terminal_settings_ = original_terminal_settings_; // make new settings same as old settings
    new_terminal_settings_.c_lflag &= ~ICANON; // disable buffered i/o
    new_terminal_settings_.c_lflag &= ~ECHO; // set echo mode
    new_terminal_settings_.c_cc[VMIN] = 1; //minimum of number input read.
    tcsetattr(0, TCSANOW, &new_terminal_settings_);
    tv_.tv_sec = 0;
    tv_.tv_usec = 0;
}

char kbdata::getUserInput()
{
    char c;
    FD_ZERO(&fds_);
    FD_SET(STDIN_FILENO, &fds_);
    int temp = select(STDIN_FILENO+1, &fds_, NULL, NULL, &tv_);
    if (temp <= 0)
    {
        c = 0;
        return c;
    }
    else
    {
        // c = fgetc(stdin);
        c = getchar();
        return c;
    }

}


int main(int argc, char **argv)
{
    ofstream flog;
    flog.open("logdata.txt");
    if(!flog.is_open())
    {
        cerr<<"file open error"<<endl;
        return -1;
    }

    flog<<"TC PZ LC RC LD RD "<<endl;

    m_serial.init();
    ros::init(argc, argv, "COM_Landing");
    ros::NodeHandle n;

    // publish bot data
    ros::Publisher botmsg_pub = n.advertise<COM_Landing::cleanBot_msg>("/CleanBot/BotMsg",1);
    // message
    COM_Landing::cleanBot_msg BotMsg;

    kbdata kb;

    //ros::Publisher pub_dsp = n.advertise<COM_Landing::DSP_msg>("/dsp_msg", 10);

    //ros::Subscriber sub_motionz = n.subscribe("/motion_z/msg", 1, motionzMsgCallback);

    printf("Running...\n");

    uint8_t PZ;
    uint16_t RC,LC,TC;
    int16_t LD,RD;

    int16_t PH,ZX,CS;	//tx data
    int sendcnt=0;    

    ros::Rate loop_rate(100);
    uint16_t precnt=0;
    while(ros::ok())
    {
        //read data
        m_serial.Read();
        if (m_serial.isReadOK == 1)
        {
            m_serial.isReadOK = 0;
            uint8_t chks=0;
            for(int i=2;i<PKGSIZE;i++)
            {
                if(i<PKGSIZE-1)
                    chks += m_serial.readbuff[i];
                else
                {
                    if(chks != m_serial.readbuff[i])
                    {
                        cerr<<endl<<endl<<"checksum error!!!!!"<<endl<<endl;
                    }
                }

                //printf("%2x ",m_serial.readbuff[i]);
            }
            //printf("\n");
            //printf("Recieved 8 bytes. %x\n", m_serial.readbuff[1]);

            // parse the data

            PZ = m_serial.readbuff[6];
            RC = (m_serial.readbuff[7]<<8)|m_serial.readbuff[8];
            LC = (m_serial.readbuff[9]<<8)|m_serial.readbuff[10];
            TC = (m_serial.readbuff[11]<<8)|m_serial.readbuff[12];
            LD = (m_serial.readbuff[13]<<8)|m_serial.readbuff[14];
            RD = (m_serial.readbuff[15]<<8)|m_serial.readbuff[16];
            printf("%2x %d %d %d %d %d\n",PZ,RC,LC,TC,LD,RD);

        }
        else
            cout<<"read not ok"<<endl;

        //publish data
        BotMsg.header.stamp = ros::Time::now();
        BotMsg.TC = TC;
        BotMsg.PZ = PZ;
        BotMsg.LC = LC;
        BotMsg.RC = RC;
        BotMsg.LD = LD;
        BotMsg.RD = RD;
        botmsg_pub.publish(BotMsg);

        uint16_t curcnt = m_serial.readbuff[11];
        curcnt = curcnt*256 + m_serial.readbuff[12];
        cout<<curcnt<<" ";
        if(precnt!=curcnt-1)
            cout<<endl<<endl<<precnt<<" precnt!=curcnt-1"<<curcnt<<endl<<endl;
        precnt = curcnt;

        //get keyboard input
        char c = kb.getUserInput();
        //cout<<c<<endl;
        switch(c)
        {
            case 'w':
            case 'W':
                PH = 166;
                ZX = -5;
                CS = PH+ZX;
                sendcnt=0;
                break;
            case 's':
            case 'S':
                PH = -166;
                ZX = -5;
                CS = PH+ZX;
                sendcnt=0;
                break;
            case 'a':
            case 'A':
                PH = 0;
                ZX = 100;
                CS = PH+ZX;
                sendcnt=0;
                break;
            case 'd':
            case 'D':
                PH = 0;
                ZX = -100;
                CS = PH+ZX; 
                sendcnt=0;
                break;
            case ' ':
                //sendcnt++;
                //if(sendcnt>80)
                //{              
                PH = 0;
                ZX = 0;
                CS = PH+ZX;
                //sendcnt=80;
                //}
                break;
            case '-':
            case '_':
                ZX++;
                CS = PH+ZX;
                break;	
            case '=':
            case '+':
                ZX--;
                CS = PH+ZX;
                break;
            case '[':
            case '{':
                PH++;
                CS = PH+ZX;
                break;
            case ']':
            case '}':
                PH--;
                CS = PH+ZX;
                break;
            case ',':
            case '<':
                PH=50;
                ZX=50;
                CS = PH+ZX;
                break;
            case '.':
            case '>':
                PH=50;
                ZX=-50;
                CS = PH+ZX;
                break;
            default:
                break;


        }

        //write command
        m_serial.writeCmd(PH,ZX,CS);

        printf("%d %d\n",PH,ZX);

        //for(int i=0;i<10;i++)
        //{
        //printf("%2x ",m_serial.ch[i]);
        //}
        //cout<<endl;

        //log the data
        flog<<TC<<" "<<((int)PZ)<<" "<<LC<<" "<<RC<<" "<<LD<<" "<<RD<<" "<<PH<<" "<<ZX<<endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    flog.close();

    return 0;
}


//void poseMsgCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
//{
//m_serial.writeDestPos(pose_msg);
//}

//void cmdMsgCallback(const std_msgs::Int8::ConstPtr& cmd_msg)
//{
//m_serial.writeCmd(cmd_msg);
//}

//// Msg from DSP to PC
//void createDSPMsg()
//{
//if (m_serial.readbuff[0] != 0xFB)
//return;

//dsp_msg.type = m_serial.readbuff[1];
//dsp_msg.len = m_serial.readbuff[2];
//for (int i = 0; i < 5; ++i)
//{
//dsp_msg.frame[i] = m_serial.readbuff[i+3];
//}

//}

//double pretime=0;

//void motionzMsgCallback(const geometry_msgs::Point32::ConstPtr& motionz_msg)
//{
//m_serial.writeMotionz(motionz_msg);


//// double sendtime = motionz_msg->z;
//// double curtime = ros::Time::now().toSec();
//// printf("curtime: %.3f\n",curtime);
//// printf("sendtime: %.3f\n",sendtime);
//// cout<<"latency= "<<curtime-sendtime<<endl;
//// cout<<"interval = "<<curtime - pretime<<endl;
//// pretime = curtime;
//}


//void motionztimecallback(const std_msgs::Time::ConstPtr& latency_msg)
//{
//ros::Time sendtime = latency_msg->data;
//ros::Time curtime = ros::Time::now();
//printf("curtime: %.4f\n",curtime.toSec());
//printf("sendtime:%.4f\n",sendtime.toSec());
//printf("latency= %.4f\n",curtime.toSec()-sendtime.toSec());
//}

