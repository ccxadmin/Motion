using System.Runtime.InteropServices;
using System;


namespace TestConsole.References
{

    public class lhmtc_ecat
    {
		//资源类型
        public const short RES_NONE                         = -1;
		public const short RES_LIMIT_POSITIVE               = 0;
		public const short RES_LIMIT_NEGATIVE               = 1;
		public const short RES_ALARM                        = 2;
		public const short RES_HOME                         = 3;
		public const short RES_GPI                          = 4;
		public const short RES_ARRIVE                       = 5;
		public const short RES_MPG                          = 6;

		public const short RES_ENABLE                       = 10;
		public const short RES_CLEAR                        = 11;
		public const short RES_GPO                          = 12;

		public const short RES_DAC                          = 20;
		public const short RES_STEP                         = 21;
		public const short RES_PULSE                        = 22;
		public const short RES_ENCODER                      = 23;
		public const short RES_ADC                          = 24;

		public const short RES_AXIS                         = 30;
		public const short RES_PROFILE                      = 31;
		public const short RES_CONTROL                      = 32;
		public const short RES_PID							= 33; 	//新增PID
		public const short RES_AUENCODER                    = 34;  	//新增辅助编码器


		public const short STEP_DIR						    = 0;
		public const short STEP_PULSE						= 1;
		//#define STEP_PWM						 2

		public const short PT_MODE_STATIC					= 0;
		public const short PT_MODE_DYNAMIC					= 1;

		public const short PT_SEGMENT_NORMAL				= 0;
		public const short PT_SEGMENT_EVEN					= 1;
		public const short PT_SEGMENT_STOP					= 2;

		public const short GEAR_MASTER_PROFILE			= 2;
		public const short GEAR_MASTER_ENCODER			= 1;
		public const short GEAR_MASTER_AXIS			= 3;

		public const short FOLLOW_ENCODER 				= 1;			//跟随编码器值
		public const short	FOLLOW_PROFILE				= 2;			//跟随规划值
		public const short	FOLLOW_AXIS					= 3;
		public const short FOLLOW_AXIS_PROFILE          = 4;			//跟随主轴规划输出值
		public const short	FOLLOW_AXIS_ENCODER			= 5;			//跟随主轴编码器输出值

		public const short FOLLOW_EVENT_PASS			= 2;
		public const short FOLLOW_EVENT_START			= 1;

		public const short FOLLOW_SEGMENT_NORMAL        = 0;
		public const short FOLLOW_SEGMENT_EVEN			= 1;
		public const short FOLLOW_SEGMENT_STOP			= 2;
		public const short FOLLOW_SEGMENT_CONTINUE      = 3;

		//Auto Home
		public const short CAPTURE_MODE_NONE					= 0;
		public const short CAPTURE_MODE_HOME					= 1;
		public const short CAPTURE_MODE_INDEX					= 2;
		public const short CAPTURE_MODE_PROBE					= 3;
		public const short CAPTURE_MODE_HOME_INDEX             = 4;

		public const short CAPTURE_STS_NONE					= 0;
		public const short CAPTURE_STS_TRIGGER					= 1;
		public const short CAPTURE_STS_SET						= 2;
		public const short CAPTURE_STS_TRIG_TIMEOUT            = 3;
		public const short CAPTURE_STS_TRIG_TWICE              = 4;

		public const short HOME_STAGE_START					= 100;
		public const short HOME_STAGE_SEARCH_LIMIT             = 101;
		public const short HOME_STAGE_SEARCH_LIMIT_ESCAPE      = 102;
		public const short HOME_STAGE_SEARCH_HOME              = 103;
		public const short HOME_STAGE_SEARCH_INDEX             = 104;
		public const short HOME_STAGE_GO_HOME					= 105;
		public const short HOME_STAGE_END						= 106;

		public const short HOME_ERROR_NONE						= 0;
		public const short HOME_ERROR_NO_LIMIT					= -1;
		public const short HOME_ERROR_AXIS_MAP					= -2;
		public const short HOME_ERROR_LIMIT_MODE               = -3;
		public const short HOME_ERROR_INDEX_DIR                = -4;
		public const short HOME_ERROR_HOME_MODE                = -5;
		public const short HOME_ERROR_NO_HOME					= -6;
		public const short HOME_ERROR_NO_INDEX					= -7;
		public const short HOME_ERROR_NO_STAGE					= -8;
		public const short HOME_ERROR_FOLLOW					= -9;

		public const short HOME_MODE_LIMIT						= 10;
		public const short HOME_MODE_LIMIT_HOME                = 11;
		public const short HOME_MODE_LIMIT_INDEX               = 12;
		public const short HOME_MODE_LIMIT_HOME_INDEX          = 13;
		public const short HOME_MODE_HOME						= 20;
		public const short HOME_MODE_HOME_INDEX                = 22;
		public const short HOME_MODE_INDEX						= 30;

		public const short COMPARE_MODE_1D						= 1;
		public const short COMPARE_MODE_2D						= 2;
		public const short COMPARE_MODE_3D						= 3;
		public const short COMPARE_MODE_4D						= 4;

		public const short COMPARE_OUT_PULSE					= 0;
		public const short COMPARE_OUT_LEVEL					= 1;

		public const short COMPARE_ERROR_NONE					= 0;
		public const short COMPARE_ERROR_FIFO_EMPTY            = 10;
		public const short COMPARE_ERROR_SEND_FIFO             = 11;
		public const short COMPARE_ERROR_FIFO_OVERFLOW         = 12;
		public const short COMPARE_ERROR_BUF_OVERFLOW          = 13;

		public const short COMPARE_CMP2D_TOTAL_NUM             = 6; //位置比较最多有6路
		public const short COMPARE_BLOW_TOTAL_NUM              = 16; //吹气最多支持16个吹气口
		
        #region lhmtc_ecat接口中用到的结构体
        /*运动模式*/
        public enum MotionMode
        {
            enum_prfmode_trap = 0,
            enum_prfmode_jog,
            enum_prfmode_s,
            enum_prfmode_pt,
            enum_prfmode_pvs,
            enum_prfmode_interpolation,
            enum_prfmode_gear,
            enum_prfmode_follow
        }
        /*点位模式运动参数*/
        public struct TrapPrfPrm
        {
            public double acc;
            public double dec;
            public double velStart;
            public short smoothTime;
        }
        /*JOG模式运动参数*/
        public struct JogPrfPrm
        {
            public double acc;
            public double dec;
            public double smooth;
        }
        /*PID参数*/
        public struct PidParam
        {
            public double kp;
            public double ki;
            public double kd;
            public double kvff;
            public double kaff;
        }

        /*插补运动坐标系参数*/
        [System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
        public struct CrdCfg
        {
            /// short
            public short dimension;
            /// short[8]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 8, ArraySubType = System.Runtime.InteropServices.UnmanagedType.I2)]
            public short[] profile;
            /// short
            public short setOriginFlag;
            /// int[8]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 8, ArraySubType = System.Runtime.InteropServices.UnmanagedType.I4)]
            public int[] orignPos;
            /// short
            public short evenTime;
            /// double
            public double synVelMax;
            /// double
            public double synAccMax;
            /// double
            public double synDecSmooth;
            /// double
            public double synDecAbrupt;
        }

        [System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
        public struct CrdBufOperation
        {
            public ushort delay;                         // 延时时间
            public short doType;                        // 缓存区IO的类型,0:不输出IO
            public ushort doAddress;					 // IO模块地址
            public ushort doMask;                        // 缓存区IO的输出控制掩码
            public ushort doValue;                       // 缓存区IO的输出值
            public short dacChannel;					 // DAC输出通道
            public short dacValue;					     // DAC输出值
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 2, ArraySubType = System.Runtime.InteropServices.UnmanagedType.U2)]
            public ushort[] dataExt;               // 辅助操作扩展数据
        }


        //前瞻缓冲区；与前瞻相关的数据结构
        public struct CrdBlockData
        {
            public short iMotionType;                             // 运动类型,0为直线插补,1为2D圆弧插补,2为3D圆弧插补,6为IO,7为延时，8位DAC
            public short iCirclePlane;                            // 圆弧插补的平面;XY—1，YZ-2，ZX-3
            public short arcPrmType;							   // 1-圆心表示法；2-半径表示法
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = System.Runtime.InteropServices.UnmanagedType.I4)]
            public int[] lPos;            // 当前段各轴终点位置

            public double dRadius;                                // 圆弧插补的半径
            public short iCircleDir;                             // 圆弧旋转方向,0:顺时针;1:逆时针
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] dCenter;                             // 2维圆弧插补的圆心相对坐标值，即圆心相对于起点位置的偏移量
            // 3维圆弧插补的圆心在用户坐标系下的坐标值
            public int height;								   // 螺旋线的高度
            public double pitch;	// 螺旋线的螺距
            //double[3]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] beginPos;
            //double[3]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] midPos;
            //double[3]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 3, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] endPos;
            //double[3][3]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 9, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] R_inv;
            //double[3][3]
            [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst = 9, ArraySubType = System.Runtime.InteropServices.UnmanagedType.R8)]
            public double[] R;

            public double dVel;                                   // 当前段合成目标速度
            public double dAcc;                                   // 当前段合成加速度
            public short loop;
            public short iVelEndZero;                             // 标志当前段的终点速度是否强制为0,值0——不强制为0;值1——强制为0
            public CrdBufOperation operation;
            public double dVelEnd;                                // 当前段合成终点速度
            public double dVelStart;                              // 当前段合成的起始速度
            public double dResPos;                                // 当前段合成位移量

        }

        //位置比较
        public struct TMDComparePrm
        {
            public short encx;             //轴号
            public short ency;
            public short encz;
            public short enca;
            public short source;          //比较源： 0：规划   1:反馈
            public short outputType;      //输出方式：0：脉冲  1：电平
            public short startLevel;      //起始电平
            public short time;            //比较输出脉冲上升沿宽度   单位100us
            public short maxerr;          //比较范围最大误差
            public short threshold;       //最优算法阈值
            public short pluseCount;      //输出脉冲个数
            public short spacetime;       //脉冲下降沿宽度
            public short delaytime;       //输出延时时间
        };
        public struct TMDCompareData
        {
            public int px;              //比较位置
            public int py;
            public int pz;
            public int pa;
        };

        public struct TMDCompareDataEX
        {
            public int px;              //比较位置
            public int py;
            public int pz;
            public int pa;
            public short time;           //上升沿宽度  单位100us
            public short spacetime;      //下降沿宽度
            public short delaytime;      //延时时间
            public short pluseCount;     //脉冲个数
        };
        #endregion

        /*初始化部分***********************************************************************************************************/

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_Open(short channel, short param, short cardNum);
		/***********************************************************************************
		LH_ResetFpga  指令说明      ： 复位主站通信功能
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 1 个参数，参数的详细信息如下。
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： LH_Open指令之后可直接调用，复位后若要操作控制卡需要重新连接，
									   可复位硬件，正常运行情况下初始化情况下调用一次即可
									   总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ResetFpga(short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_Close(short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_Reset(short cardNum);

		/***********************************************************************************
		LH_GetSlaveIds  指令说明    ： 获取从站VID和PID
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 5 个参数，参数的详细信息如下。
									   pVid                 存放VID值的指针，应在应用层分配足够空间 
									   pPid                 存放PID值的指针，应在应用层分配足够空间
									   pRevision            驱动器固件版本， 应在应用层分配足够空间
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： LH_Open指令之后可直接调用，不用先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetSlaveIds(out int pVid, out int pPid, short slaveNum, short cardNum);
		/***********************************************************************************
		LH_GetSlaveNum  指令说明    ： 获取从站个数
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 2 个参数，参数的详细信息如下。
									   sNumber              从站数量
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： LH_Open指令之后可直接调用，不用先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetSlaveNum(out short sNumber, short cardNum);
		/***********************************************************************************
		LH_GetServoNumber  指令说明 ： 获取从站中伺服驱动器的个数
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 2 个参数，参数的详细信息如下。
									   sNumber              从站数量
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetServoNumber(out short sNumber, short cardNum);
		/***********************************************************************************
		LH_GetDiNumber  指令说明    ： 获取从站中数字输入端口的通道数
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 2 个参数，参数的详细信息如下。
									   sNumber              数字输入通道数
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetDiNumber(out short sNumber, short cardNum);
		/***********************************************************************************
		LH_GetDoNumber  指令说明    ： 获取从站中数字输出端口的通道数
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 2 个参数，参数的详细信息如下。
									   sNumber              数字输出通道数
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetDoNumber(out short sNumber, short cardNum);
		/***********************************************************************************
		LH_ConnectECAT  指令说明    ： 建立通讯，完成从站的扫描
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 1 个参数，参数的详细信息如下。
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 调用前先保证调用LH_Open和LH_ResetFpga返回成功
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ConnectECAT(short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ConnectECAT_ReCount(short cardNum);
		/***********************************************************************************
		LH_DisconnectECAT  指令说明 ： 关闭通讯
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 1 个参数，参数的详细信息如下。
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 关闭通讯后，则断开了与从站的连接，一般在退出情况下调用
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_DisconnectECAT(short cardNum);
		[DllImport("lhmtc.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_LoadConfig(string pFile,short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetSlaveStatus(ref int CrcErrorCount, short cardNum);
		
        //SDO下载
		/***********************************************************************************
		LH_EcatSDODownload  指令说明： SDO下载
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 5 个参数，参数的详细信息如下。
									   axis                 从站号，从1开始
									   index                SDO 的 index
									   subindex             SDO 的 subindex
									   data                 下载的数据指针
									   data_size            下载的数据量
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_EcatSDODownload(short axis, short index, short subindex, uint data, short data_size, short cardNum);
		/***********************************************************************************
		LH_EcatSDOUpload  指令说明  ： SDO上载
		指令类型                    ： 立即指令，调用后立即生效。
		指令参数                    ： 该指令共有 5 个参数，参数的详细信息如下。
									   axis                 从站号，从1开始
									   index                对象索引
									   subindex             子索引
									   data_size            对象长度，以字节为单位
									   pBuf                 用来保存从站返回数据
									   count                缓冲区数据个数（多少个unsigned long的数据）
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatSDOUpload(short axis, short index, short subindex, short data_size, out int pBuf, short count, short cardNum);
        //设置驱动器模式
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatSetOperationMode(short axis, short mode, short cardNum);
		//读取驱动器模式
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetOperationMode(short axis, out short mode, short cardNum);
		//设置目标对象值
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatSetTargetVelocity(short axis, int velocity, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatSetTargetTorque(short axis, short torque, short cardNum);
		
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetActualVelocity(short axis, out int pVelocity, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetActualTorque(short axis, out short pTorque, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetDcLinkVoltage(short axis, out int pVoltage, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetErrorCode(short axis, out short pCode, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetStatusWord(short axis, out short pStatus, short cardNum);

		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatGetDigitalInputs(short axis, out int pDiValue, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EcatSetDigitalOutputs(short axis, int DoValue, short cardNum);
		
		////////////////////////////////////////
        //设置驱动器模式，模式6为回零模式，模式8为CSP模式
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetHomingMode(short axis, short mode, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetEcatLimitAndHomeValue(short axis, uint negValue, uint posValue, uint homeValue, short cardNum);
		//开始ECAT回零
		/***********************************************************************************
		LH_StartEcatHoming  指令说明： 启动回零
		指令类型                    ： 立即指令，（此函数通信过程大概 2-3 秒）。
		指令参数                    ： 该指令共有 8 个参数，参数的详细信息如下。
									   axis                 从站号，从1开始
									   method               设置回零模式
									   offset               原点偏移量
									   speed1               搜索开关速度。单位：pulse/ms
									   speed2               搜索 index 速度。单位：pulse/ms
									   acc                  搜索加速度。单位:pulse/ms^2
									   probeFunction        默认为 0
									   cardNum              卡号，索引从1开始
		指令返回值                  ： short                0-成功，非0失败 
		备注                        ： 使用之前一定要先调用LH_ConnectECAT
									   总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_StartEcatHoming(short axis, short method, int offset, uint speed1, uint speed2, uint acc,  ushort probeFunction, short cardNum);
		//获取驱动器编码器的绝对位置
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetEcatEncPos(short encoder,out double pValue,short count, short cardNum);
		//获取驱动器回零的状态
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetEcatHomingStatus(short axis,out short phomingStatus, short cardNum);
		//回零取消
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_HomeCancel(uint  mask, short cardNum);

        /*轴基本操作*************************************************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_AxisOn(short axis, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_AxisOff(short axis, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPrfPos(short profile, int prfPos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ZeroPos(short axis, short count, short cardNum);

        //系统参数配置
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPidIndex(short control, short index, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPidIndex(short control, out short pIndex, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPid(short control, short index, ref PidParam pPid, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPid(short control, short index, out PidParam pPid, short cardNum);


        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPosErr(short control, int error, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPosErr(short control, out int pError, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetCtrlMode(short axis, short mode, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetCtrlMode(short axis, out short mode, short cardNum);

        //软限位
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetSoftLimit(short axis, int positive, int negative, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetSoftLimit(short axis, out int pPositive, out int pNegative, short cardNum);


        //io停止加速度
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetStopDec(short axis, double dSmoothDec, double dEmergencyDec, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetStopDec(short axis, out double dSmoothDec, out double dEmergencyDec, short cardNum);
       
 
        /*运动状态检测指令************************************************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetSts(short axis, out int pSts, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ClrSts(short axis, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPrfMode(short profile, out int pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPrfPos(short profile, out double pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPrfVel(short profile, out double pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPrfAcc(short profile, out double pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_Stop(short mask, short option, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_StopEx(int  mask1_32, int option1_32,int mask33_64,int option33_64, short cardNum);
        //点位运动指令
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfTrap(short profile, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetTrapPrm(short profile, ref TrapPrfPrm pPrm, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetTrapPrm(short profile, out TrapPrfPrm pPrm, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPos(short profile, int pos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPos(short profile, out int pPos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetVel(short profile, double vel, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetVel(short profile, out double pVel, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_Update(int mask, short cardNum);
		/***********************************************************************************
		LH_UpdateEx		指令说明	： 启动点位运动或者Jog运动 
		指令类型              		： 立即指令，调用后立即生效。
		指令参数              		： 该指令共有 4 个参数，参数的详细信息如下。 
									   mask_1_16     按位指示需要启动运动的轴,当 bit 位为 1 时表示启动对应的轴。1-16轴
									   mask_17_32    按位指示需要启动运动的轴,当 bit 位为 1 时表示启动对应的轴。17-32轴
									   mask_33_48    按位指示需要启动运动的轴,当 bit 位为 1 时表示启动对应的轴。33-48轴
									   cardNum 卡号  默认为1
		指令返回值            		： 请参照指令返回值列表。 
		备注                  		： 总线卡支持该功能					
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_UpdateEx(int mask_1_16, int mask_17_32,int mask_33_48,short cardNum);
		
		//同时操作多个轴到板卡
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfTrapEx(short profile, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetTrapPrmEx(short profile, ref TrapPrfPrm pPrm, short count, short cardNum);
        
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPosEx(short profile, ref int pos, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPosEx(short profile, out int pPos, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetVelEx(short profile, ref double vel, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetVelEx(short profile, out double pVel, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfJogEx(short profile, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetJogPrmEx(short profile, ref JogPrfPrm pPrm, short count, short cardNum);

        //Jog指令
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfJog(short profile, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetJogPrm(short profile, ref JogPrfPrm pPrm, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetJogPrm(short profile, out JogPrfPrm pPrm, short cardNum);

        //访问数字IO
		/***********************************************************************************
		LH_SetEcatDo         指令说明： 设置数字 IO 输出状态。
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 3 个参数，参数的详细信息如下。 
									addr                 取值范围[1-16]  ,以16DO为一个模块，地址从1-16取值。
									value                设置的输出值，按位表示输出值，取【0-15】位，为一组
														 超过16个输出的时候，要定义value数组。每次取value的【0-15】为的值
														 内存由使用者分配
									count                以16个DO为一个模块，count表示设置多少个模块
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 总线卡支持该功能
		***********************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetEcatDo(short addr, ref int Value, short count, short cardNum);
		/***********************************************************************************
		LH_SetEcatDoBit  指令说明： 按位设置数字 IO 输出状态。
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 3 个参数，参数的详细信息如下。 
									addr                 取值范围[1-16]  ,以16DO为一个模块，地址从1-16取值。
									doIndex              索引值,取值范围【1-16】
									doValue              电平值，取值范围【0-1】
									
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetEcatDoBit(short addr, short doIndex, short Value, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetEcatDo(short addr, out uint Value, short count, short cardNum);

		/***********************************************************************************
		LH_GetEcatDi     指令说明： 读取数字 IO 输入状态
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 3 个参数，参数的详细信息如下。 
									addr                正整数，取值范围[1-32]  16个DI作为一个模块，按照DI的数量设置该地址
									pValue              数字 IO 输入状态,按位表示指定输入状态
														内存由使用者分配，以16个DI为一个模块
									count               指定读取回来的模块的数量，以16个DI为一个模块
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetEcatDi(short addr, out uint pValue, short count,short cardNum);
		
		//访问扩展模块
		//访问扩展数字IO
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetExtendDi(short address, out int pValue, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetExtendDiBit(short address, short diIndex, ref short value, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetExtendDo(short address, int value, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetExtendDoBit(short address, short doIndex, short value, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetExtendDo(short address, out int pValue, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetExtendCardCount(short count, short cardNum); //设置IO扩展板的个数
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetDoBitReverse(short doType, short doIndex, short value, short reverseTime, short cardNum);
		/************************************************************************/
		/* LH_SetExtDoBitReverse指令功能：设置指定输出IO状态，并在指定数量的控制周期后反转回来
		   address：扩展板地址1-16；
		   doIndex：数字量输出的索引；
		   value：  输出的值 0-1
		   reverseTime：翻转周期的个数（单位1ms，根据扩展IO的数量不同，有所不同），
		   cardNum:卡号
		   备注： 由于扩展IO是走的串行总线，所以其刷新周期实际上是大于1ms的。
				  例如有一个16DIDO的扩展板子，其刷新一次的周期就是1ms，如果有两个16DIDO扩展板子
				  刷新一次的周期就是2ms，依次类推，IO翻转的时间则为reverseTime*（刷新周期）*/
		/************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetExtDoBitReverse(short address,short diIndex,ref int pReverseCount,short count,short cardNum);
		/************************************************************************/
		/* LH_GetExtDiReverseCount指令功能：获得扩展输入状态变化的次数
		   address：扩展板地址1-16；
		   diIndex：数字量输入的索引；
		   pReverseCount：  返回扩展输入状态变化次数
		   count：默认为1，可以批量获取输入的翻转次数
		   cardNum:卡号
		   备注： 总线卡暂不支持*/
		/************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetExtDiReverseCount(short address,short diIndex,out int pReverseCount,short count,short cardNum);
		/************************************************************************/
		/* LH_SetExtDiReverseCount指令功能：设置扩展输入状态变化的次数
		   address：扩展板地址1-16；
		   diIndex：数字量输入的索引；
		   pReverseCount：  扩展输入状态变化次数
		   count：默认为1，可以批量获取输入的翻转次数
		   cardNum:卡号
		   备注： 可以根据需要设定，常用于清零
				  总线卡暂不支持*/
		/************************************************************************/
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetExtDiReverseCount(short address,short diIndex,ref int pReverseCount,short count,short cardNum);
		
		//操作本地的8路DIO,总线卡本地有8路的DIO
		/***********************************************************************************
		LH_SetLocalDo    指令说明： 设置数字 本地IO 输出状态。
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 2 个参数，参数的详细信息如下。 
									value                设置的输出值， 
									----输出状态按位指示 IO 输出电平.8路有效
							
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 总线卡本地带DIO操作的支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetLocalDo(int value,short cardNum);
		/***********************************************************************************
		LH_SetLocalDoBit 指令说明： 按位设置本地数字 IO 的输出状态
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 3 个参数，参数的详细信息如下。 
									doIndex             设置I/O通道号，取值范围[1,16]
									value               设置的输出值，取值范围[0-1]
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 带本地DIO的总线卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetLocalDoBit(short doIndex,short value,short cardNum);//
		/***********************************************************************************
		LH_GetLocalDo    指令说明： 读取数字本地 IO 的输出状态
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 3 个参数，参数的详细信息如下。 
									
									pValue               IO 输出状态，即 LH_SetLocalDo 指令的输出值 
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 脉冲卡支持该功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetLocalDo(out int pValue,short cardNum);
		/***********************************************************************************
		LH_GetLocalDi    指令说明： 读取本地数字 IO 输入状态
		指令类型                 ： 立即指令，调用后立即生效。
		指令参数                 ： 该指令共有 2 个参数，参数的详细信息如下。 
									pValue              按位数字 IO 输入状态，
									cardNum             卡号  默认为1
		指令返回值               ： short               0-成功，非0失败 
		备注                     ： 脉冲卡支持该功能
									总线卡支持该功能
									建议总线卡情况下，调用LH_GetEcatDi实现相同功能
		***********************************************************************************/
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetLocalDi(out int pValue,short cardNum);
		//////////////////////
		//模拟量相关接口
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetDacNumber(out short sNumber, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetAdcNumber(out short sNumber, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetDac(short channel, ref short pValue, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetDac(short channel, ref short pValue, short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetAdc(short channel, ref short pValue, short count, short cardNum);
		/////////////////////


        //访问编码器
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetEncPos(short encoder, out double pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetEncVel(short encoder, out double pValue, short count, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetEncPos(short encoder, int encPos, short cardNum);

         //PT模式
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfPt(short profile, short mode, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PtSpace(short profile, out short pSpace, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PtData(short profile, int pos, int time, short type, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PtClear(short profile, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PtStart(uint mask, uint option, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPtLoop(short profile, int loop, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPtLoop(short profile, out int loop, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetPtMemory(short profile, short memory, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetPtMemory(short profile, out short memory, short cardNum);

        //Gear 运动
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfGear(short profile, short dir, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetGearMaster(short profile, short masterindex, short masterType, short masterItem, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetGearMaster(short profile, out short masterindex, out short masterType, out short masterItem, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetGearRatio(short profile, int masterEven, int slaveEven, int masterSlope, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetGearRatio(short profile, out int masterEven, out int slaveEven, out int masterSlope, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GearStart(uint mask, short cardNum);

        //Follow模式
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_PrfFollow(short profile, short dir, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetFollowMaster(short profile, short masterIndex, short masterType, short masterItem, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetFollowMaster(short profile, out short MasterIndex, out short MasterType, out short MasterItem, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetFollowLoop(short profile, short loop, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetFollowLoop(short profile, out int pLoop, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetFollowEvent(short profile, short even, short masterDir, int pos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetFollowEvent(short profile, out short pEvent, out short pMasterDir, out int pPos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_FollowSpace(short profile, out short pSpace, short count, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_FollowData(short profile, int masterSegment, int slaveSegment, short type, short fifo, bool resend);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_FollowClear(short profile, short count, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_FollowStart(uint mask, uint option, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_FollowSwitch(uint mask, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetFollowMemory(short profile, short memory, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetFollowMemory(short profile, out short pMemory, short cardNum);

        //插补
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetCrdPrm(short crd, ref CrdCfg pCrdPrm, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetCrdPrm(short crd, out CrdCfg pCrdPrm, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdSpace(short crd, out int pSpace, short count, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdClear(short crd, short count, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_LnXY(short crd, int x, int y, double synVel, double synAcc, double velEnd, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_LnXYZ(short crd, int x, int y, int z, double synVel, double synAcc, double velEnd, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_LnXYZA(short crd, int x, int y, int z, int a, double synVel, double synAcc, double velEnd, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcXYR(short crd, int x, int y, double radius, short circleDir, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcYZR(short crd, int y, int z, double radius, short circleDir, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcZXR(short crd, int z, int x, double radius, short circleDir, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcXYC(short crd, int x, int y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcYZC(short crd, int y, int z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcZXC(short crd, int z, int x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetLastCrdPos(short crd, out int position, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufIO(short crd, ushort address, ushort doMask, ushort doValue, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufDelay(short crd, uint delayTime, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufDac(short crd, short chn, short daValue, bool bGear, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufGear(short crd, short axis, int pos, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufMove(short crd, short moveAxis, int pos, double vel, double acc, short modal, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufLmtsOn(short crd, short axis, short limitType, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufLmtsOff(short crd, short axis, short limitType, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_BufSetStopIo(short crd, short axis, short stoptype, short inputtype, short inputindex, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdStart(short mask, short option, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdStop(short mask, short option, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdStatus(short crd, out short pSts, out short pCmdNum, out int pSpace, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetCrdPos(short crd, out double pPos, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetCrdVel(short crd, out double pSynVel, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_SetCrdStopDec(short crd, double decSmoothStop, double decAbruptStop, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_GetCrdStopDec(short crd, out double decSmoothStop, out double decAbruptStop, short cardNum);

        //前瞻部分

        // x y z是终点坐标， interX, interY, interZ是中间点坐标
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_ArcXYZ(short crd, double x, double y, double z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, short fifo, short cardNum);


        //基于2维圆弧半径加终点的输入方式的螺旋线插补  xyz是终点坐标 终点坐标要跟螺距信息匹配
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineXYR(short crd, int x, int y, int z, double radius, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineYZR(short crd, int y, int z, int x, double radius, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineZXR(short crd, int z, int x, int y, double radius, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);

        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        //基于2维圆弧圆心和终点的输入方式的螺旋线插补	xyz是终点坐标 终点坐标要跟螺距信息匹配
        public static extern short LH_HelicalLineXYC(short crd, int x, int y, int z, double xCenter, double yCenter, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineYZC(short crd, int y, int z, int x, double yCenter, double zCenter, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineZXC(short crd, int z, int x, int y, double zCenter, double xCenter, short circleDir, double pitch, double synVel, double synAcc, short fifo, short cardNum);

        //基于空间圆弧的螺旋线插补，主要输入参数为定义螺旋线圆柱底面的圆弧的两个点（加上当前起点构成三点圆弧），螺旋线的高度（有正负号，根据右手定则判断螺旋线虚拟z轴的正向），螺旋线的螺距（正数），函数会自动计算螺旋线的终点，用户需要有终点停在哪里的意识
        // x y z是终点坐标， interX, interY, interZ是中间点坐标        
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_HelicalLineXYZ(short crd, double x, double y, double z, double interX, double interY, double interZ, int height, double pitch, double synVel, double synAcc, double velEnd, short fifo, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_InitLookAhead(short crd, short fifo, double T, double accMax, short n, ref CrdBlockData pLookAheadBuf, short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_CrdData(short crd, ref CrdBlockData pCrdData, short fifo, short cardNum);
		
		//等间距补偿
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetLeadScrewComp(short axis,short n,int startPos,int lenPos,ref int pCompPos,ref int pCompNeg, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_EnableLeadScrewComp(short axis,short mode, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetCompPrfPos(short axis,out double pValue,short count, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetCompEncPos(short axis,out double pValue,short count, short cardNum);
		
		//反向间隙补偿
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_SetBacklash(short axis, int compValue, double compChangeValue, int compDir, short cardNum);
		[DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern short LH_GetBacklash(short axis, out int pCompValue, out double pCompChangeValue, out int pCompDir, short cardNum);
		
		//位置比较功能
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareMode(short chn,short dimMode,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDComparePulse(short chn, short level, short outputType, short time,int lPluseCount,short spacetime,short delayTime,short cardNum);
        //chn:通道0-3   chnMode:0-脉冲引脚  1-方向引脚    Hsio:HSIO触发 0-1   delayTime:延时时间   time：上升沿宽度 单位100us    spacetime：下降沿宽度  pluseCount：输出个数 1-255
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDComparePulseEx(short chn, short chnMode, short hsio, short delayTime, short time,short spacetime,short pluseCount,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDComparePulseExStop(short chn, short chnMode,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareBindOn(short chn, short time,int lPluseCount,short spacetime,short inputIo,short senselevel,short delayTime,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)] 
        public static extern short LH_MDCompareGetBindPrm(short chn, out short bind,out short time,out int lPluseCount,out short spacetime,out short inputIo,out short senselevel,out short delayTime,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareBindOff(short chn,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareSetPrm(short chn, ref TMDComparePrm pPrm,short mode ,short cardNum);  // mode: 0-LH_MDCompareData   ;1 - LH_MDCompareDataEx
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)] 
        public static extern short  LH_MDCompareData(short chn, short count, ref  TMDCompareData pBuf, short fifo ,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short  LH_MDCompareDataEx(short chn, short count,ref TMDCompareDataEX pBuf, short fifo ,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareClear(short chn,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareStatus(short chn, out short pStatus,out int pCount, out short pFifo, out short pFifoCount, out short pBufCount,out int pTriggerPos,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareStart(short chn,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short LH_MDCompareStop(short chn,short cardNum);
        [DllImport("lhmtc.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern short  LH_SetComparePort(short chn, short hsio0, short hsio1,short cardNum);
    }

}
