using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.ComponentModel;
using System.IO;
using MotionLib;
using TestConsole.References;

namespace TestConsole
{
    /// <summary>
    /// 崧智，部分范例
    /// </summary>
    public class AOS_Pluse_MotionCard : MotionControlBase,IPluseIO
    {
        /// <summary>
        /// 脉冲卡
        /// </summary>
        public AOS_Pluse_MotionCard()
        {
            ProductSN = "MCC-P004";

        }
        /// <summary>
        /// 脉冲卡
        /// productSN：产品型号
        /// </summary>
        /// <param name="productSN">产品型号</param>
        public AOS_Pluse_MotionCard(string productSN)
        {
            ProductSN = productSN;

        }

        /// <summary>
        /// 获取产品制造商
        /// </summary>
        /// <returns>产品制造商</returns>
        public override string getProductManufacturer()
        {
            return "崧智";
        }


        /********************************运动控制合成方法***************************/
        /// <summary>
        /// 初始化控制卡
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="obj"></param>
        /// <returns></returns>
        override public  StuExecuteStatus initMotionCard<T>(T obj)
        {
            short rtn = 0;
            StuExecuteStatus stuExecuteStatus;
            if (obj == null)
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
            try
            {
                // 0 表示通道号，1为默认参数，1为卡号
                rtn = lhmtc.LH_Open(obj.Channel, 1, obj.CardNum);
                Thread.Sleep(5);
                //复位运动控制器
                rtn = lhmtc.LH_Reset(obj.CardNum);

                string m_Path = (obj as MotionCardInitParams).m_Path;
                //加载配置文件
                rtn = lhmtc.LH_LoadConfig(m_Path, obj.CardNum);

                //清除不良状态
                //一次性最多清除8个轴的状态
                short startAxis = obj.StartAxisNo;
                short usingAxisCount = obj.UsingAxisCount;
                rtn = lhmtc.LH_ClrSts(startAxis, usingAxisCount, obj.CardNum);

                //指定轴位置清零
                rtn = lhmtc.LH_ZeroPos(startAxis, usingAxisCount, obj.CardNum);


                //初始化自动回原点功能
                rtn = lhmtc.LH_HomeInit(obj.CardNum);

                stuExecuteStatus.runStatus = rtn == 0 ? stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess :
                                        EumRunStatus.Execute_InitCard_Error;

                m_bInited = stuExecuteStatus.StatusFlag = rtn == 0;

            }
            catch(Exception er)
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;

            }
            return stuExecuteStatus;

        }
        /// <summary>
        /// 关闭控制卡，释放资源
        /// </summary>
        /// <returns></returns>
        override public StuExecuteStatus releaseMotionCard<T>(T obj)
        {
            StuExecuteStatus stuExecuteStatus;
            if (!this.m_bInited)
            {
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
                stuExecuteStatus.StatusFlag = true;
                return stuExecuteStatus;
            }
            //关闭板卡
            short rtn = lhmtc.LH_Close((obj as MotionData.CommonCardParams).CardNum);
            stuExecuteStatus.StatusFlag = rtn == 0;
            if (!stuExecuteStatus.StatusFlag)
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteFail;
            else
            {
                m_bInited = false;
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
            }
            return stuExecuteStatus;
        }
        /// <summary>
        /// 轴回原
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="obj"></param>
        /// <returns></returns>
        override public  StuExecuteStatus executeCommandOfHome<T>(T obj)
        {
            if (obj == null) return default;
            short rtn = 0;
            StuExecuteStatus stuExecuteStatus;
            //定义运动控制指令返回值

            ushort status;
            int sts;
            //判断控制卡是否初始化成功
            if (!m_bInited)
            {
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteFail;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
            try
            {
                MotionCardHomeParmas mh = obj as MotionCardHomeParmas;

                //获得单个控制轴状态
                rtn = lhmtc.LH_GetSts(mh.AxisNum, out sts, 1, mh.CardNum);
                //清除单个控制轴状态
                rtn = lhmtc.LH_ClrSts(mh.AxisNum, 1, mh.CardNum);

                Thread.Sleep(5);

                rtn = lhmtc.LH_AxisOn(mh.AxisNum, mh.CardNum);

                Thread.Sleep(10);
                //单个轴位置清零 
                rtn = lhmtc.LH_ZeroPos(mh.AxisNum, 1, mh.CardNum);

                //获得单个控制轴状态
                rtn = lhmtc.LH_GetSts(mh.AxisNum, out sts, 1, mh.CardNum);

                //启动自动回原点功能，（偏移量--人为机器定义的零点）
                rtn = lhmtc.LH_Home(mh.AxisNum, mh.searchPos, mh.homeVel, mh.homeAcc,
                             mh.zeroOffset, mh.CardNum);

                Thread.Sleep(2);
                int t1 = Environment.TickCount;
                while (true)
                {
                    //查询自动回原点的运行状态 
                    //查询的状态值 
                    //0：自动回原点操作正在执行中。
                    //1：自动回原点操作成功执行完毕。
                    //2：执行完毕，未触发。 
                    //3：未到达 Home位置电机已停止 
                    rtn = lhmtc.LH_HomeSts(mh.AxisNum, out status, mh.CardNum);

                    if (status == 1)//判断是否正在运行
                    {
                        stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
                        stuExecuteStatus.StatusFlag = true;
                        return stuExecuteStatus;
                    }
                    else if (status == 2 || status == 3)
                    {
                        stuExecuteStatus.runStatus = EumRunStatus.Execute_Home_Error;
                        stuExecuteStatus.StatusFlag = false;
                        return stuExecuteStatus;
                    }
                    else
                        ;
                  
                    Thread.Sleep(1);
                }
            }
            catch
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }

        }
        /// <summary>
        /// 点位运动(绝对)
        /// </summary>
        /// <typeparam name="T">数据类型隶属于TrapParmas</typeparam>
        /// <param name="obj">参数对象</param>
        /// <returns>命令执行状态</returns>
        override public StuExecuteStatus executeCommandOfTrap_Abs<T>(T obj)
        {
            if (obj == null) return default;
            short rtn = 0;//返回值
            short m_currentPrfMode = 0; //当前运动模式
            StuExecuteStatus stuExecuteStatus;
            //定义运动控制指令返回值
            lhmtc.TrapPrfPrm TrapPrfPrm;
            TrapPrfPrm.acc = obj.ACC;
            TrapPrfPrm.dec = obj.DEC;
            TrapPrfPrm.smoothTime = (obj as MotionCardTrapParmas).smoothTime;
            TrapPrfPrm.velStart = obj.velStart;
                 
            //判断控制卡是否初始化成功
            if (!m_bInited)
            {
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteFail;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
            try
            {
                rtn = lhmtc.LH_GetPrfMode(obj.AxisNum, out m_currentPrfMode, 1, obj.CardNum);
                if (rtn == 0)
                {
                    if (m_currentPrfMode != 0)
                    {
                        //如果不是点位模式则设置为点位模式
                        rtn = lhmtc.LH_PrfTrap(obj.AxisNum, obj.CardNum);                       
                    }
                }
                //设置点位运动的参数
                rtn = lhmtc.LH_SetTrapPrm(obj.AxisNum, ref TrapPrfPrm, obj.CardNum);

                //设置目标位置
                rtn = lhmtc.LH_SetPos(obj.AxisNum, (int)obj.dPos_abs, obj.CardNum);

                //设置目标速度
                rtn = lhmtc.LH_SetVel(obj.AxisNum, obj.VelTarget, obj.CardNum);

                //启动(注意;该方法默认起始轴编号为1)
                rtn = lhmtc.LH_Update(1 << (obj.AxisNum - 1), obj.CardNum);          
                if(rtn==0)
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
                }                 
                else
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.Execute_Trap_Error;
                }
                return stuExecuteStatus;
            }
            catch
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
        }

        /// <summary>
        /// 点位运动(相对)
        /// </summary>
        /// <typeparam name="T">数据类型隶属于TrapParmas</typeparam>
        /// <param name="obj">参数对象</param>
        /// <returns>命令执行状态</returns>
        override public StuExecuteStatus executeCommandOfTrap_Rel<T>(T obj)
        {
            if (obj == null) return default;
            short rtn = 0;//返回值
            short m_currentPrfMode = 0; //当前运动模式
            StuExecuteStatus stuExecuteStatus;
            //定义运动控制指令返回值
            lhmtc.TrapPrfPrm TrapPrfPrm;
            TrapPrfPrm.acc = obj.ACC;
            TrapPrfPrm.dec = obj.DEC;
            TrapPrfPrm.smoothTime = (obj as MotionCardTrapParmas).smoothTime;
            TrapPrfPrm.velStart = obj.velStart;

            //判断控制卡是否初始化成功
            if (!m_bInited)
            {
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteFail;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
            try
            {
                rtn = lhmtc.LH_GetPrfMode(obj.AxisNum, out m_currentPrfMode, 1, obj.CardNum);
                if (rtn == 0)
                {
                    if (m_currentPrfMode != 0)
                    {
                        //如果不是点位模式则设置为点位模式
                        rtn = lhmtc.LH_PrfTrap(obj.AxisNum, obj.CardNum);
                    }
                }
                //设置点位运动的参数
                rtn = lhmtc.LH_SetTrapPrm(obj.AxisNum, ref TrapPrfPrm, obj.CardNum);

                double C_Pos = 0;
                //获取当前位置
                rtn = lhmtc.LH_GetEncPos(obj.AxisNum, out C_Pos, 1, 1);                                           
            
              int   VaribdPos = (int)obj.dPos_rel + (int)C_Pos;
                //设置目标位置
                rtn = lhmtc.LH_SetPos(obj.AxisNum, VaribdPos, obj.CardNum);

                //设置目标速度
                rtn = lhmtc.LH_SetVel(obj.AxisNum, obj.VelTarget, obj.CardNum);

                //启动(注意;该方法默认起始轴编号为1)
                rtn = lhmtc.LH_Update(1 << (obj.AxisNum - 1), obj.CardNum);
                if (rtn == 0)
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
                }
                else
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.Execute_Trap_Error;
                }
                return stuExecuteStatus;
            }
            catch
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
        }

        /// <summary>
        /// JOG运动
        /// </summary>
        /// <typeparam name="T">数据类型隶属于JogParmas</typeparam>
        /// <param name="obj">参数对象</param>
        /// <returns>命令执行状态</returns>
        override public StuExecuteStatus executeCommandOfJog<T>(T obj)
        {
            if (obj == null) return default;
            short rtn = 0;//返回值
            short m_currentPrfMode = 0; //当前运动模式
            StuExecuteStatus stuExecuteStatus;
            //定义运动控制指令返回值
            lhmtc.JogPrfPrm JogPrfPrm0;
            JogPrfPrm0.acc = obj.ACC;
            JogPrfPrm0.dec = obj.DEC;
            JogPrfPrm0.smooth= (obj as MotionCardJogParmas).smooth;
         
            //判断控制卡是否初始化成功
            if (!m_bInited)
            {
                stuExecuteStatus.runStatus = EumRunStatus.ExecuteFail;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
            try
            {
                rtn = lhmtc.LH_GetPrfMode(obj.AxisNum, out m_currentPrfMode, 1, obj.CardNum);
                if (rtn == 0)
                {
                    if (m_currentPrfMode != 1)
                    {
                        //如果不是点位模式则设置为点位模式
                        rtn = lhmtc.LH_PrfJog(obj.AxisNum, obj.CardNum);//若返回值为1则轴还在运动中，必须先停止轴，再设置为Jog模式
                    }
                }
                //设置Jog参数
                rtn = lhmtc.LH_SetJogPrm(obj.AxisNum, ref JogPrfPrm0, 1);
             
                //设置轴速度  TargetVel单位为：pulse/ms
                rtn = lhmtc.LH_SetVel(obj.AxisNum, obj.VelTarget, obj.CardNum);

                //启动，bit0 表示 1 轴，bit1 表示 2 轴...bit7 表示 8 轴
                // 当 bit 位为 1 时启动对应轴
                //(注意;该方法默认起始轴编号为1)
                rtn = lhmtc.LH_Update(1 << (obj.AxisNum - 1), obj.CardNum);
                if (rtn == 0)
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.ExecuteSuccess;
                }
                else
                {
                    stuExecuteStatus.StatusFlag = true;
                    stuExecuteStatus.runStatus = EumRunStatus.Execute_Jog_Error;
                }
                return stuExecuteStatus;
            }
            catch
            {
                stuExecuteStatus.runStatus = EumRunStatus.Unknown_Error;
                stuExecuteStatus.StatusFlag = false;
                return stuExecuteStatus;
            }
        }



        /************************实现接口（指令集）*******************************/

        /// <summary>
        ///  设置控制卡号 ，
        ///  objs[0]:设定的卡号值，默认为1，
        ///  return:指令调用信息集
        /// </summary>
        /// <param name="objs">可变参数集</param>
        /// <returns>指令调用信息集</returns>
        override public StuFuncCallInfo OR_SetCardNum(params object[] objs)
        {
            //cardNum= (short)objs[0];         
            StuFuncCallInfo stuFuncCallInfo = base.ExecuteCMD(lhmtc.LH_SetCardNum, (short)objs[0]);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "设置控制卡号错误！";

            return stuFuncCallInfo;
        }
        /// <summary>
        /// 轴位置清零，
        /// objs[0]:起始轴编号，objs[1]:轴数量，objs[2]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令执行标志</returns>
        override public StuFuncCallInfo OR_ZeroPos(params object[] objs)
        {
            StuFuncCallInfo stuFuncCallInfo;
            short startAxis = (short)objs[0];
            short axisCount= (short)objs[1];
            short cardNum = (short)objs[2];
            stuFuncCallInfo.rtn = lhmtc.LH_ZeroPos(startAxis, axisCount,cardNum);
            stuFuncCallInfo.runFlag = stuFuncCallInfo.rtn == 0;
            stuFuncCallInfo.errMsg = stuFuncCallInfo.runFlag ? "" :
                             string.Format("指令执行异常，错误代码：{0}！", stuFuncCallInfo.rtn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "轴位置清零指令错误！";

            return stuFuncCallInfo;

        }
        /// <summary>
        /// 读取轴运动模式 
        /// </summary>
        /// <param name="profile">起始轴号</param>
        /// <param name="pValue">运动模式</param>
        /// <param name="count">轴数</param>
        /// <param name="cardNum">卡号</param>
        /// <returns></returns>
        public StuFuncCallInfo OR_GetPrfMode(short profile, out short pValue, short count, short cardNum)
        {
            StuFuncCallInfo stuFuncCallInfo;
            
            stuFuncCallInfo.rtn = lhmtc.LH_GetPrfMode(profile,out pValue, count, cardNum);
            stuFuncCallInfo.runFlag = stuFuncCallInfo.rtn == 0;
            stuFuncCallInfo.errMsg = stuFuncCallInfo.runFlag ? "" :
                             string.Format("指令执行异常，错误代码：{0}！", stuFuncCallInfo.rtn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : " 读取轴运动模式指令错误！";

            return stuFuncCallInfo;      
        }

        /// <summary>
        /// 获得控制轴状态
        /// objs[0]:起始轴编号，objs[1]:轴数量，objs[2]:卡编号
        /// </summary>
        /// <param name="pSts">32位轴状态字 </param>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_GetSts(out int pSts, params object[] objs)
        {
            StuFuncCallInfo stuFuncCallInfo;
            short startAxis = (short)objs[0];
            short axisCount = (short)objs[1];
            short cardNum = (short)objs[2];
            stuFuncCallInfo.rtn = lhmtc.LH_GetSts(startAxis, out pSts, axisCount, cardNum);
            stuFuncCallInfo.runFlag = stuFuncCallInfo.rtn == 0;
            stuFuncCallInfo.errMsg = stuFuncCallInfo.runFlag ? "" :
                             string.Format("指令执行异常，错误代码：{0}！", stuFuncCallInfo.rtn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "获得控制轴状态指令错误！";

            return stuFuncCallInfo;
          
        }

        /// <summary>
        /// 清除控制轴状态
        /// objs[0]:起始轴编号，objs[1]:轴数量，objs[2]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_ClrSts(params object[] objs)
        {
            StuFuncCallInfo stuFuncCallInfo;
            short startAxis = (short)objs[0];
            short axisCount = (short)objs[1];
            short cardNum = (short)objs[2];
            stuFuncCallInfo.rtn = lhmtc.LH_ClrSts(startAxis, axisCount, cardNum);
            stuFuncCallInfo.runFlag = stuFuncCallInfo.rtn == 0;
            stuFuncCallInfo.errMsg = stuFuncCallInfo.runFlag ? "" :
                             string.Format("指令执行异常，错误代码：{0}！", stuFuncCallInfo.rtn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "清除控制轴状态指令错误！";

            return stuFuncCallInfo;

        }
        /// <summary>
        /// 停止一个或者多轴的规划运动
        /// objs[0]:按位指示需要停止的运动轴号，
        /// objs[1]:按位指示停止方式当bit位为0时表示平滑停止对应轴，为1时表示紧急停止对应轴，
        /// objs[2]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_Stop(params object[] objs) 
        {
            StuFuncCallInfo stuFuncCallInfo;
            short mask = (short)objs[0];
            short option = (short)objs[1];
            short cardNum = (short)objs[2];
            stuFuncCallInfo.rtn = lhmtc.LH_Stop(mask, option, cardNum);
            stuFuncCallInfo.runFlag = stuFuncCallInfo.rtn == 0;
            stuFuncCallInfo.errMsg = stuFuncCallInfo.runFlag ? "" :
                             string.Format("指令执行异常，错误代码：{0}！", stuFuncCallInfo.rtn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "轴停止指令错误！";

            return stuFuncCallInfo;
        }

        /// <summary>
        /// 打开控制轴伺服使能
        ///  objs[0]:轴编号，objs[1]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_AxisOn(params object[] objs)
        {           
            StuFuncCallInfo stuFuncCallInfo = base.ExecuteCMD((short)objs[0], (short)objs[1],lhmtc.LH_AxisOn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "打开控制轴伺服使能错误！";

            return stuFuncCallInfo;
        }

        /// <summary>
        /// 关闭控制轴伺服使能 
        ///  objs[0]:轴编号，objs[1]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_AxisOff(params object[] objs)
        {
            StuFuncCallInfo stuFuncCallInfo = base.ExecuteCMD((short)objs[0], (short)objs[1], lhmtc.LH_AxisOff);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "关闭控制轴伺服使能错误！";

            return stuFuncCallInfo;
        }
        /// <summary>
        /// 设置扩展IO数量
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetExtendCardCount(params object[] objs) { return default; }
        /// <summary>
        ///  读取数字I/O输入的电平状态  
        /// </summary>
        /// <param name="pValue">返回I/O输入信号值</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetDi(out long pValue, params object[] objs) { pValue = 0; return default; }
        /// <summary>
        /// 设置数字I/O输出的电平状态 
        /// </summary>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetDo(params object[] objs) { return default; }
        /// <summary>
        /// 按位设置数字I/O的输出的电平状态 
        /// </summary>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetDoBit(params object[] objs) { return default; }
        /// <summary>
        /// 读取数字I/O的输出的电平状态
        /// </summary>
        /// <param name="pValue">返回I/O输入信号值</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetDo(out long pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 读取扩展 I/O块的输入电平状态 
        /// </summary>
        /// <param name="pValue">返回 I/O输入信号</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetExtendDi(out long pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 按照读取扩展 I/O位的输入电平状态 
        /// </summary>
        /// <param name="pValue">返回 I/O输入信号</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetExtendDiBit(out short pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 设置扩展I/O块的输出电平状态 
        /// </summary>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetExtendDo(params object[] objs) { return default; }
        /// <summary>
        /// 按位设置扩展I/O位的输出电平状态
        /// </summary>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetExtendDoBit(params object[] objs) { return default; }
        /// <summary>
        /// 读取扩展I/O块的输出电平状态
        /// </summary>
        /// <param name="pValue">返回I/O输出信号值</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetExtendDo(out long pValue, params object[] objs) { pValue = 0; return default; }

        
        
        /************************崧智控制卡初始化参数*******************************/
        /// <summary>
        /// 轴初始化
        /// </summary>
        [Serializable]
        public class MotionCardInitParams : MotionData.InitCardParams
        {
            /// <summary>
            ///  初始化轴控制基础参数， bingding公共参数
            /// </summary>       
            public MotionCardInitParams()
            {

            }
            /// <summary>
            /// 配置文件路径
            /// </summary>
            [Description("配置文件路径")]
            public string m_Path { get; set; } = Directory.GetCurrentDirectory() + "\\" + "Setconfig.cfg";

        }


        /// <summary>
        /// 轴回原参数
        /// </summary>
        [Serializable]
        public class MotionCardHomeParmas : MotionData.RunHomeParmas
        {
            /// <summary>
            /// 轴回原参数，bingding公共参数
            /// </summary>
            /// <param name="initCardParams">公共参数</param>
            public MotionCardHomeParmas(MotionData.InitCardParams initCardParams)
                                   : base(initCardParams)
            {

            }

            /// <summary>
            /// 原点偏移量
            /// </summary>
            public int zeroOffset { get; set; }
            /// <summary>
            /// 搜索速度
            /// </summary>
            public double homeVel { get; set; }
            /// <summary>
            /// 搜索加速度
            /// </summary>
            public double homeAcc { get; set; }
            /// <summary>
            /// 搜索距离，以当前位置为起点,搜索距离为正时向正方向搜索，搜索距离为负时像负方向搜索
            /// </summary>
            [DefaultValue(20000000), Description("搜索距离")]
            public int searchPos { get; set; }

            /// <summary>
            /// 属性通知
            /// </summary>
            /// <param name="name">属性名称</param>
            /// <param name="obj">属性值</param>
            override public void cardPropertyChangedEvent(string name, object obj)
            {
                base.cardPropertyChangedEvent(name, obj);
            }
            
        }

        /// <summary>
        /// 点位运动参数
        /// </summary>
        [Serializable]
        public class MotionCardTrapParmas: MotionData.TrapParmas
        {
            /// <summary>
            /// 点位运动参数，bingding公共参数
            /// </summary>
            /// <param name="initCardParams">公共参数</param>
            public MotionCardTrapParmas(MotionData.InitCardParams initCardParams)
                                   : base(initCardParams)
            {

            }

            /// <summary>
            /// 平滑时间
            /// </summary>
          public short smoothTime { get; set; } = 0;

        }


        /// <summary>
        /// 点位运动参数
        /// </summary>
        [Serializable]
        public class MotionCardJogParmas : MotionData.JogParmas
        {
            /// <summary>
            /// Jog运动参数，bingding公共参数
            /// </summary>
            /// <param name="initCardParams">公共参数</param>
            public MotionCardJogParmas(MotionData.InitCardParams initCardParams)
                                   : base(initCardParams)
            {

            }

            /// <summary>
            /// 平滑系数
            /// </summary>
            public double smooth { get; set; } = 0.5;

        }



        /************************崧智控制卡指令返回值*******************************/
        enum EumReturnCode
        {
            [Description("多线程资源繁忙")]
            Multithreaded_resource_busy=-8,
            [Description("DSP繁忙")]
            DSP_busy,
            [Description("打卡/关闭设备错误")]
            Clock_inOroff_device_error,
            [Description("读写文件错误")]
            Error_readingAndwriting_file,
            [Description("写入文件错误")]
            Error_writing_file,
            [Description("通讯错误 :校验错误")]
            verification_error,
            [Description("通讯错误 :数据长度错误")]
            data_length_error,               
            [Description("打开通道错误")]
            Open_channel_error,
            [Description("指令执行成功")]
            Instruction_execution_succeeded,
            [Description("指令执行错误")]
            Instruction_execution_error,
            [Description("版本不匹配")]
            Version_mismatch=3,
            [Description("指令参数错误")]
            Command_parameter_error = 7,
            [Description("指令不支持")]
            Instruction_not_supported = 8,
        }

    }
}
