using AgOpenGPS.Classes;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public enum TrackToolMode { None = 0, Tool = 2, Slope = 4, ToolSlope = 8};

    public class CToolTrack
    {
 
        //internal readonly bool aveLineHeading;
        public List<CToolTrk> gToolArr = new List<CToolTrk>();
  
        internal bool isbtnAddToolTrackPts = false, isbtnToolTrackStop = false, isbtnToolAtWork = false;

        public int SelectedLineNumber;

        public FormGPS formGPS;


        public CToolTrack(FormGPS formGPS)
        {
            this.formGPS = formGPS;
        }
    }

    public class CToolTrk
    {         
        public List<vecRoll> curve_sowing_Pts = new List<vecRoll>();
        public List<vecRoll> curve_Toolpivot_Pts = new List<vecRoll>();
        public string nameAB;       // of tractor AB curveline
        public double ToolHeading;  // heading of curve points
        public int countAB;        //  howManyPathsAway from AB curveline
        public int ToolOffset; //Antenna on left side is neg
        public int mode; //Antenna on left side is neg


        //constructor
        public CToolTrk()// FormGPS _f)
        {
            curve_sowing_Pts = new List<vecRoll>();
            curve_Toolpivot_Pts = new List<vecRoll>();
            countAB = 0;
            nameAB = "AB Track";
            ToolOffset = 0;
            ToolHeading = 0;
            mode = 0;
        }
    }
}

