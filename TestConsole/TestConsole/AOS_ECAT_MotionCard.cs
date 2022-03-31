using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using MotionLib;
using TestConsole.References;


namespace TestConsole
{
    /// <summary>
    /// 崧智，部分范例
    /// </summary>
    public class AOS_ECAT_MotionCard : MotionControlBase,IEcatIO
    {
        /// <summary>
        /// ECAT总线卡
        /// </summary>
        public AOS_ECAT_MotionCard()
        {
            ProductSN = "MCC-E032";

        }
        /// <summary>
        /// 脉冲卡
        /// productSN：产品型号
        /// </summary>
        /// <param name="productSN">产品型号</param>
        public AOS_ECAT_MotionCard(string productSN)
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
        public override StuExecuteStatus executeCommandOfHome<T>(T obj)
        {
            throw new NotImplementedException();
        }

        public override StuExecuteStatus executeCommandOfJog<T>(T obj)
        {
            throw new NotImplementedException();
        }

        public override StuExecuteStatus executeCommandOfTrap_Abs<T>(T obj)
        {
            throw new NotImplementedException();
        }
        public override StuExecuteStatus executeCommandOfTrap_Rel<T>(T obj)
        {
            throw new NotImplementedException();
        }

        public override StuExecuteStatus initMotionCard<T>(T obj)
        {
            throw new NotImplementedException();
        }

        public override StuExecuteStatus releaseMotionCard<T>(T obj)
        {
            throw new NotImplementedException();
        }


        /************************实现接口（指令集）*******************************/

        /// <summary>
        /// 打开控制轴伺服使能
        ///  objs[0]:轴编号，objs[1]:卡编号
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令运行状态</returns>
        override public StuFuncCallInfo OR_AxisOn(params object[] objs)
        {
            StuFuncCallInfo stuFuncCallInfo = base.ExecuteCMD((short)objs[0], (short)objs[1], lhmtc_ecat.LH_AxisOn);

            stuFuncCallInfo.errMsg += stuFuncCallInfo.runFlag ? "" : "打开控制轴伺服使能错误！";

            return stuFuncCallInfo;
        }

        /// <summary>
        /// 设置扩展IO数量
        /// </summary>
        /// <param name="objs">参数集</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetExtendCardCount(params object[] objs) { return default; }

        /// <summary>
        /// 获取从站个数
        /// </summary>
        /// <param name="pValue">返回从站个数</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetSlaveNum(out short pValue, params object[] objs) { pValue = 0; return default; }
        /// <summary>
        /// 获取从站伺服个数 
        /// </summary>
        /// <param name="pValue">返回从站伺服个数</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetServoNumber(out short pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 获取从站数字输入个数
        /// </summary>
        /// <param name="pValue">返回从站数字输入个数</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetDiNumber(out short pValue, params object[] objs) { pValue = 0; return default; }
        /// <summary>
        /// 获取从站数字输出个数
        /// </summary>
        /// <param name="pValue">返回从站数字输出个数</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetDoNumber(out short pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 按位设置数字 IO 输出状态。
        /// </summary>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_SetEcatDoBit(params object[] objs) { return default; }

        /// <summary>
        /// 读取数字 IO 输出状态
        /// </summary>
        /// <param name="pValue">数字IO输出状态,按位表示指定输入状态</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetEcatDo(out uint pValue, params object[] objs) { pValue = 0; return default; }

        /// <summary>
        /// 读取数字IO输入状态
        /// </summary>
        /// <param name="pValue">数字IO输入状态,按位表示指定输入状态</param>
        /// <param name="objs">参数对象</param>
        /// <returns>指令执行标志</returns>
        virtual public StuFuncCallInfo OR_GetEcatDi(out uint pValue, params object[] objs) { pValue = 0; return default; }



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

        /************************崧智控制卡指令返回值*******************************/
        enum EumReturnCode
        {
            [Description("无响应")]
            No_response = -7,
            [Description("打开运动控制器失败")]
            Failed_toOpen_MotionController = -6,
            [Description("通讯出错")]
            Communication_error = -1,
            [Description("指令执行成功")]
            Instruction_execution_succeeded = 0,
            [Description("指令错误")]
            Instruction_error=1,
            [Description("错误许可")]
            Wrong_license=2,
            [Description("PC写命令超时")]
            PC_write_timeout=3,
            [Description("PC读命令超时")]
            PC_read_timeout = 4,
            [Description("校验和错误")]
            Checksum_error=5,
            [Description("指令参数错误")]
            Command_parameter_error = 7,
            [Description("未知错误")]
            unknown_error = 8,
           
        }
    }
}
