using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MotionLib;

namespace TestConsole
{
    //控制台
    class Program
    {
        static void Main(string[] args)
        {
            //控制卡类型实例化
            ImotionController Controller = new AOS_Pluse_MotionCard();
          
            //卡1传递所需类型参数
            (Controller as Idata).setData<AOS_Pluse_MotionCard.MotionCardInitParams>("卡1初始化参数", 
                                                                 EumParmasType.initCardParams);
            (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").CardNum = 2;
            (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").Channel = 1;
            (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").initAxisNum(1, 4,
                new string[] { "上料",
                "下料",
                "移栽",
                "搬运"});

            (Controller as Idata).setData<AOS_Pluse_MotionCard.MotionCardHomeParmas>("上料","上料轴回原参数",
                 EumParmasType.runHomeParmas, (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数"));
            (Controller as Idata).getData<AOS_Pluse_MotionCard.MotionCardHomeParmas>("上料轴回原参数").zeroOffset = 0;

            (Controller as Idata).setData<AOS_Pluse_MotionCard.MotionCardHomeParmas>("下料","下料轴回原参数",
                 EumParmasType.runHomeParmas, (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数"));

            (Controller as Idata).setData<AOS_Pluse_MotionCard.MotionCardHomeParmas>("移栽", "移栽轴回原参数",
                 EumParmasType.runHomeParmas, (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数"));

            //自定义参数集合获取
            List<string> Parmanames = (Controller as AOS_Pluse_MotionCard).motionDataDic.Names;
            //移除不需要的轴参数
            (Controller as AOS_Pluse_MotionCard).motionDataDic.Remove("下料轴回原参数");

            (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").RemoveAxisInfo("移栽");

            (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").modifyAxisInfo( "搬运",3);
            //刷新参数集合
           (Controller as MotionControlBase).refreshMotionDataDic();

            //方法调用
            StuExecuteStatus stuExecuteStatus = Controller.initMotionCard<AOS_Pluse_MotionCard.MotionCardInitParams>
                          ((Controller as Idata).getData<AOS_Pluse_MotionCard.MotionCardInitParams>("卡1初始化参数"));

            StuExecuteStatus stuExecuteStatus2 = Controller.executeCommandOfHome<AOS_Pluse_MotionCard.MotionCardHomeParmas>
                ((Controller as Idata).getData<AOS_Pluse_MotionCard.MotionCardHomeParmas>("上料轴回原参数"));

            StuExecuteStatus stuExecuteStatus3=Controller.releaseMotionCard<AOS_Pluse_MotionCard.MotionCardInitParams>
                          ((Controller as Idata).getData<AOS_Pluse_MotionCard.MotionCardInitParams>("卡1初始化参数"));

            Console.WriteLine(stuExecuteStatus.ToString());
            Console.WriteLine(stuExecuteStatus2.ToString());
            Console.WriteLine(stuExecuteStatus3.ToString());

            //参数文件保存加载
            (Controller as IFileOperate).SaveParameterFile((Controller as MotionControlBase).motionDataDic);
            (Controller as MotionControlBase).motionDataDic = (Controller as IFileOperate).LoadParameterFile<userDictionary<string, EumParmasType, IParmas>>();
            List<string> axisFuncNameList = (Controller as Idata).getData<MotionData.InitCardParams>("卡1初始化参数").axisFuncNameList;
            foreach (var s in axisFuncNameList)
                Console.WriteLine(s);

            Console.ReadKey();
        }
    }
}
