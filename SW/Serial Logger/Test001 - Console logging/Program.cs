using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.IO.Ports;

namespace Test001___Console_logging
{
    class Program
    {
        static void Main(string[] args)
        {
            string buff = null;
            SerialPort com = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
            com.Open();
            while(true)
            {
                buff = com.ReadLine();
                if(buff != null)
                {
                    Console.WriteLine(buff);
                }
            }
        }
    }
}
