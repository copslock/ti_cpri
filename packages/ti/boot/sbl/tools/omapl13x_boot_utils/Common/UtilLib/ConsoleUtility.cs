/*
 * ConsoleUtility.cs
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/

/* 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/****************************************************************
 *  TI UtilLib.ConsoleUtility namespace: Useful Console Utils   *
 *  (C) 2007, Texas Instruments, Inc.                           *
 *                                                              *
 * Author:  Daniel Allred                                       *
 *                                                              *
 ****************************************************************/

using System;
using System.Text;

namespace TI.UtilLib.ConsoleUtility
{
  public enum Position : uint
  {
    NONE = 0,
    TOP,
    BOTTOM,
    RIGHT,
    LEFT,
  }
    
    public class ProgressBar
    {
        #region Private Variables

        private Int32 posYBar, posXBar;
        private Int32 posYPercent,posXPercent;
        private Int32 posYText, posXText;
        private Int32 barWidth, totalWidth;
        private Position percPosition, textPosition;
        private Double percent;
        private String text;

        #endregion

        #region Private Properties

        private Position PercentPosition
        {
          get
          {
            return percPosition;
          }
          set
          {
            if ((value == Position.BOTTOM) || (value == Position.TOP))
            {
                percPosition = Position.LEFT;
            }
            else
            {
                percPosition = value;
            }
          }
        }

        private Position TextPosition
        {
          get
          {
            return textPosition;
          }
          set
          {
            if ((value == Position.RIGHT) || (value == Position.LEFT))
            {
              textPosition = Position.BOTTOM;
            }
            else
            {
              textPosition = value;
            }
          }
        }

        #endregion

        #region Public Properties

        /// <summary>
        /// Set the percentage completeness of the progress bar.
        /// </summary>
        public Double Percent
        {
          get
          {
            return percent;
          }
          set
          {
            percent = (value > 1) ? 1 : value;
            UpdatePercent();
            if (percent == 1)
            {
              Console.CursorVisible = true;
            }
          }
        }

        /// <summary>
        /// Set the status text field
        /// </summary>
        public String Text
        {
          get
          {
            return text;
          }
          set
          {
            text = value;
            UpdateText();
          }
        }
        #endregion

        #region Class Constructors
        /// <summary>
        /// Default constructor
        /// </summary>
        public ProgressBar():this(60)
        {
        }

        /// <summary>
        /// Variable-width ProgressBar constructor
        /// </summary>
        /// <param name="charWidth">Width of progress bar.</param>
        public ProgressBar(Int32 charWidth): this(charWidth,Position.LEFT,Position.BOTTOM)
        {
        }

        /// <summary>
        /// Variable-width and variable percent location ProgressBar constructor
        /// </summary>
        /// <param name="charWidth">Width of status bar in characters.</param>
        /// <param name="pos">StatusPosition enum value.</param>
        /// <param name="percentPos">Position enum for Percentage location.</param>
        /// <param name="txtPos">Position enum for the text location.</param>
      
        public ProgressBar(Int32 charWidth, Position percentPos, Position textPos)
        {
          Int32 prevCursorTop;
          
          // Make sure we are starting on a clean, new line           
          if (Console.CursorLeft != 0)
          {
            Console.WriteLine();
          }
          
          // Set private member values
          PercentPosition = percentPos;
          TextPosition = textPos;
          barWidth = charWidth;
          percent = 0.0;
          
          posYText    = -1;  posXText    = -1;
          posYPercent = -1;  posXPercent = -1;
          posYBar     = -1;  posXBar     = -1;
          
          //Make the cursor invisible
          Console.CursorVisible = false;

          // Place initial fields
          if (TextPosition == Position.TOP)
          {
            posYText = Console.CursorTop;
            posXText = Console.CursorLeft;
            prevCursorTop = Console.CursorTop;
            Console.WriteLine("Status Undefined");
            if (Console.CursorTop == prevCursorTop)
            {
              // This indicates that we hit the end of the buffer and 
              // that everything scrolled up
              posYText = (posYText<0) ? posYText : (posYText-1);
              posYPercent = (posYPercent<0) ? posYPercent : (posYPercent-1);
              posYBar = (posYBar<0) ? posYBar : (posYBar-1);
            }
          }
          if (PercentPosition == Position.LEFT)
          {
            posXPercent = Console.CursorLeft;
            posYPercent = Console.CursorTop;
            Console.Write(" {0,3:D}% ", 0);
          }
          
          Console.Write("[ ");
          posYBar = Console.CursorTop;
          posXBar = Console.CursorLeft;
          for (int i = 0; i < barWidth; i++)
          {
            Console.Write("-");
          }
          Console.Write(" ]");
          
          if (PercentPosition == Position.RIGHT)
          {
            posXPercent = Console.CursorLeft;
            posYPercent = Console.CursorTop;                
            Console.Write(" {0,3:D}% ", 0);
          }
          totalWidth = Console.CursorLeft;

          if (Console.CursorLeft != 0)
          {
            prevCursorTop = Console.CursorTop;
            Console.WriteLine();
            if (Console.CursorTop == prevCursorTop)
            {
              // This indicates that we hit the end of the buffer and 
              // that everything scrolled up
              posYText = (posYText<0) ? posYText : (posYText-1);
              posYPercent = (posYPercent<0) ? posYPercent : (posYPercent-1);
              posYBar = (posYBar<0) ? posYBar : (posYBar-1);
            }
          }
          if (TextPosition == Position.BOTTOM)
          {
            posYText = Console.CursorTop;
            posXText = Console.CursorLeft;
            prevCursorTop = Console.CursorTop;
            Console.WriteLine("Status Undefined");
            if (Console.CursorTop == prevCursorTop)
            {
              // This indicates that we hit the end of the buffer and 
              // that everything scrolled up
              posYText = (posYText<0) ? posYText : (posYText-1);
              posYPercent = (posYPercent<0) ? posYPercent : (posYPercent-1);
              posYBar = (posYBar<0) ? posYBar : (posYBar-1);
            }
          }

          //Go to a new line so any future text can be placed
          prevCursorTop = Console.CursorTop;
          Console.WriteLine();
          if (Console.CursorTop == prevCursorTop)
          {
            // This indicates that we hit the end of the buffer and 
            // that everything scrolled up
            posYText = (posYText<0) ? posYText : (posYText-1);
            posYPercent = (posYPercent<0) ? posYPercent : (posYPercent-1);
            posYBar = (posYBar<0) ? posYBar : (posYBar-1);
          }
        }
        #endregion

        #region Class methods

        #region Public Methods
        public void Update(Double percent, String status)
        {
          Percent = percent;
          Text = "".PadLeft(totalWidth,Char.Parse(" "));
          Text = status;
        }
        
        public void WriteLine(String text)
        {
          this.Write(text+"\n");
        }
        
        public void Write(String text)
        {
          Int32 prevCursorTop;
          Int32 newLineCount = 0;
          
          //Count newlines in text to figure out how much buffer will scroll
          newLineCount = text.Split(Char.Parse("\n")).Length - 1;
          
          prevCursorTop = Console.CursorTop;
          Console.Write(text);
          if ((Console.CursorTop - prevCursorTop) < newLineCount)
          {
            Int32 shiftValue = newLineCount - (Console.CursorTop - prevCursorTop);
            // This indicates that we hit the end of the buffer and 
            // that everything scrolled up
            posYText = (posYText<0) ? posYText : (posYText-shiftValue);
            posYPercent = (posYPercent<0) ? posYPercent : (posYPercent-shiftValue);
            posYBar = (posYBar<0) ? posYBar : (posYBar-shiftValue);
          }
        }
        
        
        #endregion
        
        #region Public Static Methods
        public static void WriteLine(ProgressBar pb, String text)
        {
          pb.WriteLine(text);
        }
        
        public static void Write(ProgressBar pb, String text)
        {
          pb.Write(text);
        }
        
        #endregion

        #region Private helper functions
        private void UpdatePercent()
        {
            // Save current cursor position
            int oldCursorTop = Console.CursorTop, oldCursorLeft = Console.CursorLeft;
            
            // Update the percent if needed
            if (PercentPosition != Position.NONE)
            {
                Console.SetCursorPosition(posXPercent,posYPercent);
                Console.Write(" {0,3:D}% ", (Int32)(Percent*100));
            }

            // Update the progress bar
            Console.SetCursorPosition(posXBar,posYBar);
            Int32 numOfBlocks = (Int32)Math.Floor(barWidth * Percent);
            Console.Write("".PadLeft(numOfBlocks,Char.Parse("\u2588")));
            
            // Reset the cursor location            
            Console.SetCursorPosition(oldCursorLeft, oldCursorTop);
        }

        private void UpdateText()
        {
            int oldCursorTop = Console.CursorTop, oldCursorLeft = Console.CursorLeft;

            // Update the text field
            if (TextPosition != Position.NONE)
            {
                Console.SetCursorPosition(posXText, posYText);
                if (text.Length < totalWidth)
                    Console.Write(text.PadLeft((totalWidth + text.Length) / 2));
                else
                    Console.Write(text);
            }
            
            // Reset the cursor location            
            Console.SetCursorPosition(oldCursorLeft, oldCursorTop);
        }
        #endregion

        #endregion

    }
}