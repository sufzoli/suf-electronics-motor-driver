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
            int i,j;

            SerialPort com = new SerialPort(
                settings.COM_Port,
                settings.COM_Baud,
                ToParity(settings.COM_Parity),
                settings.COM_Bits,
                ToStopBits(settings.COM_StopBits));
            com.Open();
            i = 0;
            j = 0;
            while (buff != ".")
            {
                i++;
                buff = com.ReadLine();
                if (buff != null)
                {
                    if(settings.Console)
                    {
                        Console.WriteLine(buff);
                    }
                    calibrationfile.WriteLine(buff);
                }
                if(i>1000)
                {
                    calibrationfile.Flush();
                    i = 0;
                }
                if (settings.MaxSamples != 0)
                {
                    if(settings.MaxSamples < j)
                    {
                        break;
                    }
                    j++;
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
