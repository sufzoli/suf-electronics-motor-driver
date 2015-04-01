using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.IO;
using System.IO.Ports;

using MotorCalibrationLogger.Properties;

namespace MotorCalibrationLogger
{
    class Program
    {
        static void Main(string[] args)
        {
            string buff = null;
            Settings settings = new Settings();
            StreamWriter calibrationfile = new StreamWriter(settings.LogFile);

            SerialPort com = new SerialPort(
                settings.COM_Port,
                settings.COM_Baud,
                ToParity(settings.COM_Parity),
                settings.COM_Bits,
                ToStopBits(settings.COM_StopBits));
            com.Open();
            while (buff != ".")
            {
                buff = com.ReadLine();
                if (buff != null)
                {
                    Console.WriteLine(buff);
                    calibrationfile.WriteLine(buff);
                }
            }
            calibrationfile.Flush();
            calibrationfile.Close();
        }
        static Parity ToParity(string parityStr)
        {
            Parity retvalue;
            switch(parityStr.ToLower())
            {
                case "n":
                    retvalue = Parity.None;
                    break;
                default:
                    retvalue = Parity.None;
                    break;
            }
            return retvalue;
        }
        static StopBits ToStopBits(string stopbitStr)
        {
            StopBits retvalue;
            switch(stopbitStr.Trim())
            {
                case "0":
                    retvalue = StopBits.None;
                    break;
                case "1":
                    retvalue = StopBits.One;
                    break;
                case "1.5":
                    retvalue = StopBits.OnePointFive;
                    break;
                case "1,5":
                    retvalue = StopBits.OnePointFive;
                    break;
                case "2":
                    retvalue = StopBits.Two;
                    break;
                default:
                    retvalue = StopBits.None;
                    break;
            }
            return retvalue;
        }
    }
}
