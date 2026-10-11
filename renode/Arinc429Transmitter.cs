//
// VoltDragon: an ARINC 429 transmitter on SPI, for Node B (HLR-015).
//
// Real boards pair the MCU with an ARINC 429 line driver controlled over SPI, such
// as Holt's HI-3593. This model stands in for one with a deliberately small command
// set of its own (not the HI-3593's opcodes; see docs/arinc429.md):
//
//   0x01                  reset: forget any partial command
//   0x02 b31..24 .. b7..0 transmit one 32-bit word, most significant byte first
//   0x03 0x00             read the identifier: the second byte returns 0xA4
//
// A frame is the bytes sent while chip select (GPIO input 0) is low. Each word is
// put on the bus at once (no FIFO timing) and decoded by a built-in bus monitor,
// which logs the label, SDI, SSM, parity and, for the labels Node B sends, the
// value. Robot tests check those log lines; the decoding here is independent of
// the firmware's encoder.
//

using System;
using System.Globalization;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class Arinc429Transmitter : ISPIPeripheral, IGPIOReceiver
    {
        public Arinc429Transmitter()
        {
            Reset();
        }

        public void Reset()
        {
            WordsTransmitted = 0;
            LastWord = 0;
            ParityErrors = 0;
            EndFrame();
        }

        public byte Transmit(byte data)
        {
            byte reply = 0x00;

            if(position == 0)
            {
                opcode = data;
                if(opcode == OpcodeReset)
                {
                    this.Log(LogLevel.Info, "arinc429: reset");
                }
            }
            else if(opcode == OpcodeTransmit && position <= 4)
            {
                word = (word << 8) | data;
                if(position == 4)
                {
                    Send(word);
                }
            }
            else if(opcode == OpcodeReadId && position == 1)
            {
                reply = Identifier;
            }
            position++;
            return reply;
        }

        public void FinishTransmission()
        {
            EndFrame();
        }

        public void OnGPIO(int number, bool value)
        {
            // Chip select is active low: raising it ends the frame.
            if(number == 0 && value)
            {
                EndFrame();
            }
        }

        public ulong WordsTransmitted { get; private set; }

        public uint LastWord { get; private set; }

        public ulong ParityErrors { get; private set; }

        private void EndFrame()
        {
            position = 0;
            opcode = 0;
            word = 0;
        }

        private void Send(uint value)
        {
            WordsTransmitted++;
            LastWord = value;

            // Bit 1 (the first transmitted) is the label's most significant bit.
            uint label = 0;
            for(int bit = 0; bit < 8; bit++)
            {
                label = (label << 1) | ((value >> bit) & 1u);
            }
            uint sdi = (value >> 8) & 0x3u;
            uint data = (value >> 10) & 0x7FFFFu;
            uint ssm = (value >> 29) & 0x3u;
            bool parityOk = CountOnes(value) % 2 == 1;
            if(!parityOk)
            {
                ParityErrors++;
            }

            string labelOctal = Convert.ToString((int)label, 8).PadLeft(3, '0');
            this.Log(LogLevel.Info, "arinc429: label {0} sdi {1} ssm {2} data 0x{3:X5} parity {4}{5}",
                labelOctal, sdi, SsmName(ssm), data, parityOk ? "ok" : "ERROR", Engineering(labelOctal, data, ssm));
        }

        // BNR: the 19 data bits are a two's complement fraction of the label's range, with the
        // sign in bit 29; the significant bits used by the source do not change the scaling.
        private static string Engineering(string label, uint data, uint ssm)
        {
            if(ssm != SsmNormal)
            {
                return "";
            }
            int signed = (data & 0x40000u) != 0 ? (int)data - 0x80000 : (int)data;
            switch(label)
            {
                case "314":
                    return Format(", true heading {0:F2} deg", signed * 180.0 / 262144.0);
                case "203":
                    return Format(", altitude {0:F0} ft", signed * 131072.0 / 262144.0);
                case "251":
                    return Format(", line distance {0:F2} m", signed * 8192.0 / 262144.0);
                default:
                    return "";
            }
        }

        private static string Format(string format, double value)
        {
            return string.Format(CultureInfo.InvariantCulture, format, value);
        }

        private static string SsmName(uint ssm)
        {
            switch(ssm)
            {
                case 0: return "FW";
                case 1: return "NCD";
                case 2: return "FT";
                default: return "NO";
            }
        }

        private static int CountOnes(uint value)
        {
            int count = 0;
            for(; value != 0; value &= value - 1)
            {
                count++;
            }
            return count;
        }

        private int position;
        private byte opcode;
        private uint word;

        private const byte OpcodeReset = 0x01;
        private const byte OpcodeTransmit = 0x02;
        private const byte OpcodeReadId = 0x03;
        private const byte Identifier = 0xA4;
        private const uint SsmNormal = 3;
    }
}
