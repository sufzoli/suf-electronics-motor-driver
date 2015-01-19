using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Drawing;

namespace FC0004
{
    public class DataCollector
    {
        public DataCollector()
        {
            this._aggregatedict = new Dictionary<uint, uint>();
            this._collecteddata = new List<uint>();
        }
        public DataCollector(UInt32 MaxDataLength)
        {
            this._maxdatalength = MaxDataLength;
            this._aggregatedict = new Dictionary<uint, uint>();
            this._collecteddata = new List<uint>();
        }
        private Dictionary<UInt32, UInt32> _aggregatedict;
        private List<UInt32> _collecteddata;
        private UInt32 _maxdatalength;
        public UInt32 MaxDataLength
        {
            get
            {
                return this._maxdatalength;
            }
            set
            {
                this._maxdatalength = value;
                // do some cleanup
                this._CleanUp();
            }
        }
        public void Add(UInt32 data)
        {
            this._collecteddata.Add(data);
            // if we have more data than the maximum allowed, remove the oldest
            this._CleanUp();
            // Add to the aggregate
            if(this._aggregatedict.ContainsKey(data))
            {
                this._aggregatedict[data]++;
            }
            else
            {
                this._aggregatedict.Add(data, 1);
            }
        }
        private void _CleanUp()
        {
            while (this._collecteddata.Count > (MaxDataLength-1))
            {
                // if the data is in the agregate
                if (this._aggregatedict.ContainsKey(this._collecteddata[0]))
                {
                    // if it has a value bigger than zero
                    if (this._aggregatedict[this._collecteddata[0]] > 0)
                    {
                        // decrease the number
                        this._aggregatedict[this._collecteddata[0]]--;
                        // if the aggregate arrived to zero
                        if (this._aggregatedict[this._collecteddata[0]] == 0)
                        {
                            // remove it entierly
                            this._aggregatedict.Remove(this._collecteddata[0]);
                        }
                        // Remove the value from the collected list
                        this._collecteddata.RemoveAt(0);
                    }
                    else
                    {
                        // something went wrong. Log the error
                        // the value can not be null when we have a data to remove from the agregate
                    }
                }
                else
                {
                    // something went wrong. Log the error
                    // the key must exists when we have data to remove from it
                }
            }
        }
        public void Render()
        {
            int width = 800;
            int height = 600;
            Bitmap BMP = new Bitmap(width, height);
            // Get the key list and sort
            UInt32[] keyarr = this._aggregatedict.Keys.ToArray();
            Array.Sort(keyarr);

        }
    }
}
