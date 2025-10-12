using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AgOpenGPS.Forms.Guidance
{
    public partial class FormToolTracks : Form
    {
        private readonly FormGPS mf = null;

        public List<CToolTrk> gToolArr = new List<CToolTrk>();

        private bool isScreenbig = false, isSlopeDirectionLeft = true;
        private bool isToolTrackexist = false, isHeadingToolSameWay = true;
        private bool isSimulatorOn = true;

        private double Tooleasting;
        private double Toolnorthing;
        private double Switchheading = 0;
        private double HeadingLinePivot = 0;

        private double nudSetToolAnthight, nudSetToolPivot;
        private double nudSetOfset, nudSetOfset1;
        private double ToolLookahead, ToolpWM;
        private double RollToolVeh = 0, ccbevor = 0;
        private double lastLat = 0;
        private double lastLong = 0;
        private double lastheading = 0;
        private double lastroll = 0;
        private double CircumLengthAB, CircumLengthBC, CircumLengthCA, CircumLength;
        private int SelectedTrackidx = 0;
        private int OriginalgArrNum;

        private int ToolXTE = 151; // in cm * 10
        private int XTEVeh = 58974;
        private int VehRoll = 65478;  // in degrees * 10

        vecRoll ToolPointNow = new vecRoll();
        vecRoll ToolLastPoint = new vecRoll();
        vecRoll PivotToolLine = new vecRoll();
        vecRoll PivotToolSlopeLine = new vecRoll();
        vecRoll VehicleSlopeLine = new vecRoll();
        vecRoll HelpPoint = new vecRoll();
        public List<vec3> SlopeTemp3 = new List<vec3>();
        public List<vecRoll> SlopeTemp4 = new List<vecRoll>();
        public List<vecRoll> SlopeTemp5 = new List<vecRoll>();


        public FormToolTracks(Form _mf)
        {
            mf = _mf as FormGPS;
            InitializeComponent();

        }

        private void FormToolTracks_Load(object sender, EventArgs e)
        {
            if (mf.trk.idx < 0)
            {
                mf.TimedMessageBox(3000, "  No AB line active    ", "   get one ");
                Close();
            }

            OriginalgArrNum = mf.trk.idx;

            ToolLookahead = Properties.Settings.Default.setTool_ToolLookahead;
            ToolpWM = Properties.Settings.Default.setTool_ToolPWM;
            nudSetOfset = Properties.Settings.Default.setTool_nudSetOfset;
            nudSetToolAnthight = Properties.Settings.Default.setTool_nudSetToolAnthight;
            nudSetToolPivot = Properties.Settings.Default.setTool_nudSetToolPivot;
            nudSetToolOffset.Value = (decimal)nudSetOfset;
            //            nudSetToolAntHight.Value = (decimal)nudSetToolAnthight;
            hsbarToolLookAhead.Value = (int)ToolLookahead;
            nudToolbehindPivot.Value = (int)nudSetToolPivot;
            hToolPWM.Value = (int)ToolpWM;

            if (nudSetOfset > 0)
            {
                rbtnToolAntennaLeft.Checked = true;
                rbtnToolAntennaRight.Checked = false;
                rbtnToolAntennaCenter.Checked = false;

            }
            else if (nudSetOfset < 0)
            {
                rbtnToolAntennaLeft.Checked = false;
                rbtnToolAntennaRight.Checked = true;
                rbtnToolAntennaCenter.Checked = false;
            }
            else if (nudSetOfset == 0)
            {
                rbtnToolAntennaLeft.Checked = false;
                rbtnToolAntennaRight.Checked = false;
                rbtnToolAntennaCenter.Checked = true;
            }


            mf.tool.isToolAntenna3 = true;
            isScreenbig = false;
            this.Size = new Size(110, 550);
            mf.FileCreateToolTrackcurve();
            mf.FileLoadToolTrackcurve();

        }

        private void AddToolTrackPts()
        {
            double minDistance = mf.avgSpeed * 0.1;  // distance of points in the new curve in m
            double maxDistance = (mf.tool.width - mf.tool.overlap) * 0.8;           //make sure point distance isn't too big
            double distance = glm.Distance(ToolPointNow, ToolLastPoint);  // last point to 3. antennapoint

            if (distance > minDistance)
            {
                vecRoll pointA = new vecRoll(ToolLastPoint.easting, ToolLastPoint.northing, mf.pivotAxlePos.heading, RollToolVeh);
                vecRoll pointB = new vecRoll(ToolPointNow.easting, ToolPointNow.northing, mf.pivotAxlePos.heading, RollToolVeh);
                pointA.heading = Math.Atan2(ToolPointNow.easting - ToolLastPoint.easting, ToolPointNow.northing - ToolLastPoint.northing);
                if (mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.Count > 1)
                {
                    int Mcount = mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.Count;
                    mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.RemoveAt(Mcount - 1);  // add 3. antennapoint to curve last point
                    mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.Add(pointA);  // add 3. antennapoint to curve last point
                }
                mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.Add(pointB);  // add new 3. antennapoint to curve
                ToolLastPoint = pointB;
                HelpPoint = pointA;
                double east = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count - 1].easting;
                double north = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count - 1].northing;
                double rollt = 0;
                vecRoll pointC = new vecRoll(east, north, pointA.heading, rollt);
                vecRoll pointD = new vecRoll(PivotToolLine.easting, PivotToolLine.northing, pointA.heading, rollt);
                if (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count > 1)
                {
                    int Mcount = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count;
                    mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.RemoveAt(Mcount - 1);  // add 3. antennapoint to curve last point
                    mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Add(pointC);  // add 3. antennapoint to curve last point
                }
                mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Add(pointD);  // add new 3. antennapoint to curve

                // Console.WriteLine("ToolLastPoint" + pointB.easting + "  " + pointB.northing + "  " + pointB.heading + "  " + pointB.toolroll);
            }
            mf.tooltrk.SelectedLineNumber = SelectedTrackidx;
            if ((mf.tooltrk.gToolArr.Count > 0) && (mf.tooltrk.gToolArr[SelectedTrackidx].curve_sowing_Pts.Count > 2))
                mf.curve.DrawToolCurve();
        }

        private void FindToolTrackXTE()
        {
            //mf.tooltrk.gToolArr[SelectedTrackidx]
            int ccPa = 0;
            double minDistAPa = 100000;

            if (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count > 4)
            {
                for (int ip = 0; ip < mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count - 1; ip++)
                {
                    double distPa = ((mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ip].easting - PivotToolLine.easting)
                    * (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ip].easting - PivotToolLine.easting))
                    + ((mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ip].northing - PivotToolLine.northing)
                    * (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ip].northing - PivotToolLine.northing));
                    if (distPa < minDistAPa)
                    {
                        minDistAPa = distPa;
                        ccPa = ip;
                    }
                }
            }
            // formula of Heron
            CircumLengthAB = glm.Distance(mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa], mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa + 1]);
            CircumLengthBC = glm.Distance(PivotToolLine, mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa + 1]);
            CircumLengthCA = glm.Distance(mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa], PivotToolLine);
            CircumLength = CircumLengthAB + CircumLengthBC + CircumLengthCA;

            ToolXTE = (int)(200 * CircumLengthAB * Math.Sqrt(CircumLength * (CircumLength - CircumLengthAB) * (CircumLength - CircumLengthBC) * (CircumLength - CircumLengthCA)));

            double guidanceToolLookDist = mf.avgSpeed * 0.277777 * ToolLookahead;
            mf.guidanceToolLookPos.easting = PivotToolLine.easting + (Math.Sin(mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa].heading) * guidanceToolLookDist);
            mf.guidanceToolLookPos.northing = PivotToolLine.northing + (Math.Cos(mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[ccPa].heading) * guidanceToolLookDist);

            isHeadingToolSameWay = Math.PI - Math.Abs(Math.Abs(mf.pivotAxlePos.heading - mf.tooltrk.gToolArr[SelectedTrackidx].ToolHeading) - Math.PI) < glm.PIBy2;
            nudSetOfset1 = nudSetOfset;
            if (!isHeadingToolSameWay)
            {
                nudSetOfset1 *= -1;
            }


        }

        private void LoadupTracks_Vehicle()
        {
            mf.tooltrk.gToolArr.Sort((a, b) => a.countAB - b.countAB);

            for (int idx = 0; idx < mf.tooltrk.gToolArr.Count; idx++)
            {
                for (int jidx = 0; jidx < mf.tooltrk.gToolArr[idx].curve_Toolpivot_Pts.Count; jidx++)
                {
                    double east = mf.tooltrk.gToolArr[idx].curve_Toolpivot_Pts[jidx].easting;
                    double north = mf.tooltrk.gToolArr[idx].curve_Toolpivot_Pts[jidx].northing;
                    double head = mf.tooltrk.gToolArr[idx].curve_Toolpivot_Pts[jidx].heading;
                }
            }

            if (mf.ct.ContourLineList.Count > 1)
            {
                mf.trk.gArr.Add(new CTrk());
                mf.trk.gArr[mf.trk.gArr.Count - 1].name = ("&Pa " + 1);
                mf.trk.gArr[mf.trk.gArr.Count - 1].mode = (int)TrackMode.Curve;
                mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts = mf.ct.ContourLineList[0];
            }
            mf.FileSaveTracks();
            Close();

        }

        private void btnDeletePattern_Click(object sender, EventArgs e)
        {
            mf.FileDeletePatternTracks();
            Close();
        }

        private void ToolAtWork()
        {
            FindToolTrackXTE();

            Console.WriteLine(" sending XTE to Tool ESP32 ");
            mf.p_233.pgn[mf.p_233.ToolXTELo] = unchecked((byte)((int)(ToolXTE)));
            mf.p_233.pgn[mf.p_233.ToolXTEHi] = unchecked((byte)((int)(ToolXTE) >> 8));
            mf.p_233.pgn[mf.p_233.ToolLookAheadLo] = unchecked((byte)((int)(ToolLookahead)));
            mf.p_233.pgn[mf.p_233.ToolLookAheadHi] = unchecked((byte)((int)(ToolLookahead) >> 8));
            int isToolAutoSteer = 0;
            if (mf.isBtnAutoSteerOn) isToolAutoSteer = 1;
            mf.p_233.pgn[mf.p_233.status] = unchecked((byte)((int)(isToolAutoSteer)));
            mf.p_233.pgn[mf.p_233.LowXTEVeh] = unchecked((byte)((int)(XTEVeh)));
            mf.p_233.pgn[mf.p_233.HighXTEVeh] = unchecked((byte)((int)(XTEVeh) >> 8));
            mf.p_233.pgn[mf.p_233.VehRollLo] = unchecked((byte)((int)(VehRoll)));
            mf.p_233.pgn[mf.p_233.VehRollHi] = unchecked((byte)((int)(VehRoll) >> 8));
            mf.SendPgnToLoop(mf.p_233.pgn);
        }

        private void btnAddToolTrackPts_Click(object sender, EventArgs e)
        {
            if ((mf.isThirdAntenne) || (mf.isSlopeline))
            {
                mf.tooltrk.isbtnAddToolTrackPts = !mf.tooltrk.isbtnAddToolTrackPts;
                mf.tooltrk.isbtnToolTrackStop = false;
                mf.tooltrk.isbtnToolAtWork = false;

                if (mf.tooltrk.isbtnAddToolTrackPts)
                {
                    mf.TimedMessageBox(3000, "Record Tool Curve  ", " started ");
                    btnToolAtWork.Image = AgOpenGPS.Properties.Resources.AutoSteerOff;
                    btnAddToolTrackPts.BackColor = Color.YellowGreen;

                    if (mf.isThirdAntenne) ToolStartRecFirstPoint();  // check a few things
                }
                else
                {
                    mf.TimedMessageBox(3000, "Record Tool Curve  ", "stopped   ");
                    btnAddToolTrackPts.BackColor = Color.Transparent;
                }
            }
            else
            {
                if (!isScreenbig) btnSetupopenclose1_Click();
                mf.TimedMessageBox(3000, "      Record ", "Tool or Slope");
            }
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            if (MessageBox.Show("Close and save tracks?", "  Are you sure? ",
                 MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                btnToolAtWork.Image = AgOpenGPS.Properties.Resources.AutoSteerOff;
                btnAddToolTrackPts.BackColor = Color.Transparent;
                Properties.Settings.Default.setTool_ToolLookahead = ToolLookahead;
                Properties.Settings.Default.setTool_ToolPWM = ToolpWM;
                Properties.Settings.Default.setTool_nudSetOfset = nudSetOfset;
                Properties.Settings.Default.setTool_nudSetToolAnthight = nudSetToolAnthight;
                Properties.Settings.Default.setTool_nudSetToolPivot = nudSetToolPivot;
                mf.tool.isToolAntenna3 = false;
                for (int icurve = 0; icurve < mf.tooltrk.gToolArr.Count; icurve++)
                {
                    if (mf.tooltrk.gToolArr[icurve].curve_Toolpivot_Pts.Count < 5)
                    {
                        mf.tooltrk.gToolArr.RemoveAt(icurve);
                    }
                }

                mf.FileSaveToolTrackcurve();
                Close();
            }
        }

        private void btnToolTrackDelete_Click(object sender, EventArgs e)
        {
            if (MessageBox.Show("Delete all Lines?", "Are you sure? ",
                MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                if (SelectedTrackidx > mf.tooltrk.gToolArr.Count)
                {
                    SelectedTrackidx = mf.tooltrk.gToolArr.Count - 1;
                    return;
                }
                if (SelectedTrackidx == 0)  // cero for delete all
                {
                    mf.tooltrk.gToolArr?.Clear();
                    mf.FileSaveToolTrackcurve();
                    Close();
                }
                else
                {
                    mf.tooltrk.gToolArr.RemoveAt(SelectedTrackidx);
                }
            }
        }

        private void btnToolAtWork_Click(object sender, EventArgs e)  // activate steering with saved ToolTrackpt
        {
            if (mf.isSlopeline)
            {
                btnAddToolTrackPts.BackColor = Color.Transparent;
                mf.tooltrk.isbtnAddToolTrackPts = false;
                mf.tooltrk.isbtnToolTrackStop = false;

                mf.tooltrk.isbtnToolAtWork = !mf.tooltrk.isbtnToolAtWork;

                if (mf.tooltrk.isbtnToolAtWork)
                {
                    mf.TimedMessageBox(3000, "Tool Steering  ", "activated ");
                    btnToolAtWork.Image = AgOpenGPS.Properties.Resources.AutoSteerOn;

                    if ((mf.isThirdAntenne) && (!mf.isSlopeline))
                    {
                        LoadupTracks_Vehicle();
                        FindactivLine();
                    }

                    if (mf.tooltrk.gToolArr.Count == 0)
                    {
                        mf.TimedMessageBox(3000, "No tracks saved  ", "           ");
                        return;
                    }
                }
                else
                {
                    mf.TimedMessageBox(3000, "Tool Steering  ", "stopped");
                    btnToolAtWork.Image = AgOpenGPS.Properties.Resources.AutoSteerOff;
                }
            }
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void FindactivLine()
        {
            // check if track exists
            if (mf.tooltrk.gToolArr.Count > 0)
            {
                for (int iSelectAB = 0; iSelectAB < mf.tooltrk.gToolArr.Count - 1; iSelectAB++)
                {
                    if (mf.tooltrk.gToolArr[iSelectAB].nameAB == mf.trk.gArr[mf.trk.idx].name)
                    {
                        for (int iCountAB = 0; iCountAB < mf.tooltrk.gToolArr.Count - 1; iCountAB++)
                        {
                            if (mf.tooltrk.gToolArr[iSelectAB].countAB == mf.curve.howManyPathsAway)
                            {
                                lastLat = mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts.Count - 1].easting;
                                lastLong = mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts.Count - 1].northing;
                                lastheading = mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts.Count - 1].heading;
                                lastroll = mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[iSelectAB].curve_Toolpivot_Pts.Count - 1].toolroll;
                                isToolTrackexist = true;
                                SelectedTrackidx = iSelectAB;
                            }
                        }
                    }
                }
            }
        }

        private void btnThirdAntenna_Click(object sender, EventArgs e)
        {
            if (!mf.isThirdAntenne)
            {
                btnThirdAntenna.BackColor = Color.GreenYellow;
                mf.isThirdAntenne = true;

                btnSlideRoll.Image = Properties.Resources.RollSlidehill_off;
                mf.isSlopeline = false;

                btnAddnewSlope.Visible = true;
                btnAddnewSlope.Text = "Tool";

            }
            else
            {
                btnThirdAntenna.BackColor = Color.WhiteSmoke;
                btnSlideRoll.Image = Properties.Resources.RollSlidehill_off;
                mf.isThirdAntenne = false;
                btnAddnewSlope.Text = "";
            }


        }

        private void btnSlideRoll_Click(object sender, EventArgs e)
        {
            if (!mf.isSlopeline)
            {
                btnSlideRoll.Image = Properties.Resources.RollSlidehill_on;
                mf.isSlopeline = true;

                btnThirdAntenna.BackColor = Color.WhiteSmoke;
                mf.isThirdAntenne = false;
                btnAddnewSlope.Text = "Slope";

                if ((mf.tooltrk.gToolArr.Count < 1) && (mf.isSlopeline)) CreatefirstSlopeLine();
                btnAddnewSlope.Visible = true;
            }
            else
            {
                btnSlideRoll.Image = Properties.Resources.RollSlidehill_off;
                mf.isSlopeline = false;
                btnAddnewSlope.Text = "";
            }

        }

        private void CreatefirstSlopeLine()   // makes origin AB to 1. slope
        {
            // convert origin curve into 1.slope 

            if ((mf.tooltrk.gToolArr.Count < 1) && (mf.isSlopeline))
            {
                double Pointdiff = 0;
                mf.tooltrk.gToolArr.Add(new CToolTrk());
                vecRoll Slopebeginn = new vecRoll();
                vec3 Slopebeginn3 = new vec3();
                double minDistA = 1000000, Extentions = 50;


                if (mf.trk.gArr[mf.trk.idx].mode == (int)TrackMode.AB)  // makes a curve from origin Line
                {
                    Slopebeginn = new vecRoll(mf.trk.gArr[mf.trk.idx].ptA.easting, mf.trk.gArr[mf.trk.idx].ptA.northing, mf.trk.gArr[mf.trk.idx].heading, RollToolVeh);

                    double A1A2Distance;

                    A1A2Distance = glm.Distance(mf.trk.gArr[mf.trk.idx].ptA, mf.trk.gArr[mf.trk.idx].ptB);

                    while (Pointdiff < A1A2Distance)
                    {
                        Pointdiff += 1;
                        Slopebeginn.easting += (Math.Sin(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        Slopebeginn.northing += (Math.Cos(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        Slopebeginn.heading = mf.trk.gArr[mf.trk.idx].heading;
                        Slopebeginn.toolroll = RollToolVeh;
                        mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Add(Slopebeginn);
                    }
                    vecRoll SlopeABLine1 = new vecRoll();
                    vecRoll SlopeABLine2 = new vecRoll();

                    SlopeABLine1.easting = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[0].easting;
                    SlopeABLine1.northing = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[0].northing;

                    int CountExtentPoints = (int)(Extentions / 1.0);  // A1A2Distance);
                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        SlopeABLine1.easting -= (Math.Sin(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        SlopeABLine1.northing -= (Math.Cos(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        SlopeABLine1.heading = mf.trk.gArr[mf.trk.idx].heading;
                        mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Insert(0, SlopeABLine1);
                    }
                    SlopeABLine2.easting = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Count - 1].easting;
                    SlopeABLine2.northing = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Count - 1].northing;

                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        SlopeABLine2.easting += (Math.Sin(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        SlopeABLine2.northing += (Math.Cos(mf.trk.gArr[mf.trk.idx].heading) * 1.0);
                        SlopeABLine2.heading = mf.trk.gArr[mf.trk.idx].heading;
                        mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Add(SlopeABLine2);
                    }

                }
                else if (mf.trk.gArr[mf.trk.idx].mode == (int)TrackMode.Curve)
                {
                    // make the 1. Slope line from the origin 
                    for (int islope = 1; islope < mf.trk.gArr[mf.trk.idx].curvePts.Count; islope++)
                    {
                        Slopebeginn = new vecRoll(mf.trk.gArr[mf.trk.idx].curvePts[islope].easting, mf.trk.gArr[mf.trk.idx].curvePts[islope].northing, mf.trk.gArr[mf.trk.idx].curvePts[islope].heading, RollToolVeh);
                        mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Add(Slopebeginn);
                    }
                }

                //move the ABLine over based on the overlap amount set in vehicle
                double distanceToMove = (0.5 * (mf.tool.width - mf.tool.overlap));
                bool isSameSide;

                //close call hit
                int cc = 0;

                for (int j = 0; j < mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Count - 1; j++)
                {
                    double dist = ((VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].easting)
                        * (VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].easting))
                                    + ((VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].northing)
                                    * (VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        cc = j;
                    }
                }

                HeadingLinePivot = Math.Atan2(VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[cc].easting, VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[cc].northing);
                HeadingLinePivot = glm.Radiant0To2Pi(HeadingLinePivot);
                isSameSide = Math.PI - Math.Abs(Math.Abs(HeadingLinePivot - (mf.trk.gArr[mf.trk.idx].heading + glm.PIBy2)) - Math.PI) < glm.PIBy2;
                if (!isSameSide)
                {
                    Switchheading = glm.Radiant0To2Pi(mf.trk.gArr[mf.trk.idx].heading - glm.PIBy2);
                    distanceToMove *= -1;
                }
                else Switchheading = glm.Radiant0To2Pi(mf.trk.gArr[mf.trk.idx].heading + glm.PIBy2);
                /*
                                Console.WriteLine("Switchheading2 " + (Switchheading * 57.2));
                                Console.WriteLine("isHeadingLinePivot " + (isHeadingLinePivot * 57.2));
                                Console.WriteLine("isSameSide " + isSameSide.ToString());
                                Console.WriteLine("distanceToMove " + distanceToMove.ToString());
                */
                for (int j = 0; j < mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Count; j++)
                {
                    Slopebeginn.easting = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].easting + (Math.Sin(mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].heading + glm.PIBy2) * distanceToMove);
                    Slopebeginn.northing = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].northing + (Math.Cos(mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].heading + glm.PIBy2) * distanceToMove);
                    Slopebeginn.heading = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].heading;
                    Slopebeginn.toolroll = mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].toolroll;
                    Slopebeginn3.easting = Slopebeginn.easting;
                    Slopebeginn3.northing = Slopebeginn.northing;
                    Slopebeginn3.heading = Slopebeginn.heading;
                    SlopeTemp4.Add(Slopebeginn);
                    SlopeTemp3.Add(Slopebeginn3);

                }

                mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Clear();
                //mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts = SlopeTemp4;
                foreach (var item in SlopeTemp4)
                {
                    mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Add(item);
                }

                mf.FileDeleteSlopeTracks();

                mf.tooltrk.gToolArr[0].ToolHeading = mf.trk.gArr[mf.trk.idx].heading;
                mf.tooltrk.gToolArr[0].nameAB = mf.trk.gArr[mf.trk.idx].name;
                mf.tooltrk.gToolArr[0].countAB = 1;
                mf.tooltrk.gToolArr[0].ToolOffset = 0;
                mf.tooltrk.gToolArr[0].mode = 4;

                // write the Slope line into the list of lines
                mf.trk.gArr.Add(new CTrk());
                mf.trk.gArr[mf.trk.gArr.Count - 1].name = ("&FIX " + 1);
                mf.trk.gArr[mf.trk.gArr.Count - 1].mode = (int)TrackMode.Curve;
                foreach (var item in SlopeTemp3)
                {
                    mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts.Add(item);
                }
                string FixName = "&FIX " + mf.tooltrk.gToolArr[SelectedTrackidx].countAB;
                SelectedTrackidx = 0;
                mf.FileSaveTracks();

                mf.trk.idx = mf.trk.gArr.Count - 1;

                SlopeTemp3.Clear();
                SlopeTemp4.Clear();

                mf.curve.DrawSlope();
            }
        }

        private void CreateAddRollinline()   // add the Roll to first slope
        {
            double minDistA = 1000000;
            int cc = 0;
            vecRoll Slopebeginn = new vecRoll();
            vec3 Slopebeginn3 = new vec3();
            SelectedTrackidx = mf.tooltrk.gToolArr.Count - 1;

            if (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count > 1)
            {
                for (int j = 0; j < mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count - 1; j++)
                {
                    double dist = ((VehicleSlopeLine.easting - mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].easting)
                        * (VehicleSlopeLine.easting - mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].easting))
                                    + ((VehicleSlopeLine.northing - mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].northing)
                                    * (VehicleSlopeLine.northing - mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        cc = j;
                    }
                }

                Slopebeginn.easting = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[cc].easting;
                Slopebeginn.northing = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[cc].northing;
                Slopebeginn.heading = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[cc].heading;
                Slopebeginn.toolroll = RollToolVeh;
                if (ccbevor != cc)
                {
                    SlopeTemp4.Add(Slopebeginn);
                    ccbevor = cc;
                }
            }
            mf.curve.DrawSlope();
        }

        private void CreatenewSlopeline()   // add new slope
        {
            vecRoll Slopebeginn = new vecRoll();
            vec3 Slopebeginn3 = new vec3();
            double widthMinusOverlap = Math.Abs((mf.tool.width - mf.tool.overlap));
            bool isSameSide = true;
            SelectedTrackidx = mf.tooltrk.gToolArr.Count - 1;

            if ((mf.tooltrk.gToolArr.Count > 0) && (mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count > 10))
            {
                SlopeTemp3.Clear();
                SlopeTemp5.Clear();

                double distanceToMove = (0.5 * (mf.tool.width - mf.tool.overlap));
                if (HeadingLinePivot == 0)
                {
                    double minDistA = 1000000;

                    //close call hit
                    int cc = 0;

                    for (int j = 0; j < mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts.Count - 1; j++)
                    {
                        double dist = ((VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].easting)
                            * (VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].easting))
                                        + ((VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].northing)
                                        * (VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[j].northing));
                        if (dist < minDistA)
                        {
                            minDistA = dist;
                            cc = j;
                        }
                    }

                    HeadingLinePivot = Math.Atan2(VehicleSlopeLine.easting - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[cc].easting, VehicleSlopeLine.northing - mf.tooltrk.gToolArr[0].curve_Toolpivot_Pts[cc].northing);
                    HeadingLinePivot = glm.Radiant0To2Pi(HeadingLinePivot);
                    isSameSide = Math.PI - Math.Abs(Math.Abs(HeadingLinePivot - (mf.trk.gArr[mf.trk.idx].heading + glm.PIBy2)) - Math.PI) < glm.PIBy2;
                    if (!isSameSide)
                    {
                        Switchheading = glm.Radiant0To2Pi(mf.trk.gArr[mf.trk.idx].heading - glm.PIBy2);
                        distanceToMove *= -1;
                    }
                    else Switchheading = glm.Radiant0To2Pi(mf.trk.gArr[mf.trk.idx].heading + glm.PIBy2);
                }

                for (int j = 0; j < mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Count; j++)
                {
                    // is next slope
                    double RollToolVehp = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].toolroll;
                    widthMinusOverlap *= Math.Abs(Math.Cos(RollToolVehp));
                    Slopebeginn.easting = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].easting + (Math.Sin(Switchheading) * widthMinusOverlap);
                    Slopebeginn.northing = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].northing + (Math.Cos(Switchheading) * widthMinusOverlap);
                    Slopebeginn.heading = mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts[j].heading;
                    Slopebeginn.toolroll = RollToolVeh;
                    SlopeTemp5.Add(Slopebeginn);     // is next slope

                    // is slope for Line list 
                    Slopebeginn3.easting = Slopebeginn.easting;
                    Slopebeginn3.northing = Slopebeginn.northing;
                    Slopebeginn3.heading = Slopebeginn.heading;
                    SlopeTemp3.Add(Slopebeginn3);
                }
                Console.WriteLine("Switchheading2 " + (Switchheading * 57.2));
                Console.WriteLine("HeadingLinePivot " + (HeadingLinePivot * 57.2));
                Console.WriteLine("isSameSide " + isSameSide.ToString());
                Console.WriteLine("distanceToMove " + distanceToMove.ToString());


                mf.tooltrk.gToolArr.Add(new CToolTrk());
                SelectedTrackidx = mf.tooltrk.gToolArr.Count - 1;
                mf.tooltrk.gToolArr[SelectedTrackidx].ToolHeading = mf.trk.gArr[mf.trk.idx].heading;
                mf.tooltrk.gToolArr[SelectedTrackidx].nameAB = mf.trk.gArr[mf.trk.idx].name;
                mf.tooltrk.gToolArr[SelectedTrackidx].countAB = SelectedTrackidx + 1;
                mf.tooltrk.gToolArr[SelectedTrackidx].ToolOffset = 0;
                mf.tooltrk.gToolArr[SelectedTrackidx].mode = 4;
                foreach (var item in SlopeTemp5)
                    mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Add(item);
                SlopeTemp5.Clear();

                // write SlopeTemp3 into the list of lines
                mf.trk.gArr.Add(new CTrk());
                mf.trk.gArr[mf.trk.gArr.Count - 1].name = ("&FIX " + mf.tooltrk.gToolArr[SelectedTrackidx].countAB);
                string FixName = "&FIX " + mf.tooltrk.gToolArr[SelectedTrackidx].countAB;
                mf.trk.gArr[mf.trk.gArr.Count - 1].mode = (int)TrackMode.Curve;
                foreach (var item in SlopeTemp3)
                    mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts.Add(item);

                SlopeTemp3.Clear();
                mf.FileSaveTracks();

                mf.trk.idx = mf.trk.gArr.Count - 1;

                // draw the active slope
                mf.curve.DrawSlope();
            }
            else
                mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Clear();
        }

        private void Calculate_All_Lines()
        {
            double widthMinusOverlap;
            double XTE_Roll = Math.Sin(RollToolVeh) * nudSetToolAnthight * 0.01;   // Rolldistance
            double XTE_Center_east = Math.Cos(HelpPoint.heading) * (nudSetOfset + XTE_Roll);  //  distance to center tractor pivot line, east
            double XTE_Center_north = Math.Sin(HelpPoint.heading) * (nudSetOfset + XTE_Roll);    // distance to center tractor pivot line, north
            widthMinusOverlap = Math.Abs((mf.tool.width - mf.tool.overlap) * (Math.Cos(RollToolVeh)));

            //  get coordinats from ESP32(UDPComm.Designer)
            if ((mf.isThirdAntenne) && (!mf.isSlopeline))
            {
                ToolPointNow.easting = mf.pn.ToolLatitude;
                ToolPointNow.northing = mf.pn.ToolLongitude;

                // line points of toolantenna 
                ToolPointNow.easting = mf.pn.ToolLatitude - Math.Cos(HelpPoint.heading) * XTE_Roll;
                ToolPointNow.northing = mf.pn.ToolLongitude + Math.Sin(HelpPoint.heading) * XTE_Roll;

                // line points middle of tool 
                PivotToolLine.easting = mf.pn.ToolLatitude - XTE_Center_east;
                PivotToolLine.northing = mf.pn.ToolLongitude + XTE_Center_north;
            }

            if ((mf.isThirdAntenne) && (mf.isSlopeline))
            {
                ToolPointNow.easting = mf.pn.ToolLatitude;
                ToolPointNow.northing = mf.pn.ToolLongitude;

                // line points of toolantenna 
                ToolPointNow.easting = mf.pn.ToolLatitude - Math.Cos(HelpPoint.heading) * XTE_Roll;
                ToolPointNow.northing = mf.pn.ToolLongitude + Math.Sin(HelpPoint.heading) * XTE_Roll;

                // line points middle of tool 
                PivotToolLine.easting = mf.pn.ToolLatitude - XTE_Center_east;
                PivotToolLine.northing = mf.pn.ToolLongitude + XTE_Center_north;


                // next line with toolwidth dependence on roll and same inclination
                widthMinusOverlap *= Math.Cos(RollToolVeh);
                PivotToolSlopeLine.easting = PivotToolLine.easting - Math.Cos(HelpPoint.heading) * widthMinusOverlap;
                PivotToolSlopeLine.northing = PivotToolLine.northing + Math.Sin(HelpPoint.heading) * widthMinusOverlap;
            }

            if ((!mf.isThirdAntenne) && (mf.isSlopeline))
            {
                PivotToolSlopeLine.easting = mf.pivotAxlePos.easting + Math.Cos(mf.pivotAxlePos.heading) * widthMinusOverlap;
                PivotToolSlopeLine.northing = mf.pivotAxlePos.northing - Math.Sin(mf.pivotAxlePos.heading) * widthMinusOverlap;
                VehicleSlopeLine.easting = mf.pivotAxlePos.easting;
                VehicleSlopeLine.northing = mf.pivotAxlePos.northing;
            }

            // position for antenna point on screen
            if (isSimulatorOn)
            {
                Tooleasting = nudSetOfset * 0.01;
                Toolnorthing = nudSetToolPivot * 0.01;

                mf.tool.AntennaToolOfset.easting = Tooleasting;
                mf.tool.AntennaToolOfset.northing = Toolnorthing;
                // back pivot
                mf.tool.pivotTool.easting = mf.tool.AntennaToolOfset.easting - nudSetOfset * 0.01;
                mf.tool.pivotTool.northing = mf.tool.AntennaToolOfset.northing;
            }
        }

        private void ToolStartRecFirstPoint()
        {
            FindactivLine();

            if (!isToolTrackexist)
            {
                mf.tooltrk.gToolArr.Add(new CToolTrk());
                SelectedTrackidx = mf.tooltrk.gToolArr.Count - 1;
                vecRoll pointbeginn = new vecRoll(mf.pn.ToolLatitude, mf.pn.ToolLongitude, mf.pivotAxlePos.heading, RollToolVeh);
                mf.tooltrk.gToolArr[SelectedTrackidx].curve_Toolpivot_Pts.Add(pointbeginn);
                mf.tooltrk.gToolArr[SelectedTrackidx].ToolHeading = mf.pivotAxlePos.heading;
                mf.tooltrk.gToolArr[SelectedTrackidx].nameAB = mf.trk.gArr[mf.trk.idx].name;
                mf.tooltrk.gToolArr[SelectedTrackidx].countAB = (int)mf.curve.howManyPathsAway;
                mf.tooltrk.gToolArr[SelectedTrackidx].ToolOffset = (int)nudSetOfset;
                ToolLastPoint = pointbeginn;
                isToolTrackexist = false;
            }
            else
            {
                vecRoll pointlast = new vecRoll(lastLat, lastLong, lastheading, lastroll);
                ToolLastPoint = pointlast;
            }
        }

        private void nudSetToolAntHight1_ValueChanged(object sender, EventArgs e)
        {

        }

        private void nudSetToolAntHight1_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            nudSetToolAnthight = (double)nudSetToolAntHight1.Value;
        }

        private void nudToolbehindPivot_ValueChanged(object sender, EventArgs e)
        {

        }

        private void label3_Click(object sender, EventArgs e)
        {

        }

        private void groupBox5_Enter(object sender, EventArgs e)
        {

        }

        private void lblToolPWM_Click(object sender, EventArgs e)
        {

        }

        private void lblToolLookAhead_Click(object sender, EventArgs e)
        {

        }

        private void label10_Click(object sender, EventArgs e)
        {

        }

        private void label9_Click(object sender, EventArgs e)
        {

        }

        private void label8_Click(object sender, EventArgs e)
        {

        }

        private void label7_Click(object sender, EventArgs e)
        {

        }

        private void hToolPWM_Scroll(object sender, ScrollEventArgs e)
        {

        }

        private void hsbarToolLookAhead_Scroll(object sender, ScrollEventArgs e)
        {

        }

        private void label6_Click(object sender, EventArgs e)
        {

        }

        private void label5_Click(object sender, EventArgs e)
        {

        }

        private void nudSetToolOffset_ValueChanged(object sender, EventArgs e)
        {

        }

        private void label4_Click(object sender, EventArgs e)
        {

        }

        private void nudSetToolOffset_Click(object sender, EventArgs e)
        {
            if (mf.KeypadToNUD((NudlessNumericUpDown)sender, this))
            {
                if ((double)nudSetToolOffset.Value == 0)
                {
                    rbtnToolAntennaLeft.Checked = false;
                    rbtnToolAntennaRight.Checked = false;
                    rbtnToolAntennaCenter.Checked = true;
                    nudSetOfset = 0;
                }
                else
                {
                    if (!rbtnToolAntennaLeft.Checked && !rbtnToolAntennaRight.Checked)
                        rbtnToolAntennaRight.Checked = true;

                    if (rbtnToolAntennaRight.Checked)
                        nudSetOfset = (double)nudSetToolOffset.Value;
                    else
                        nudSetOfset = (double)nudSetToolOffset.Value;
                }

                Properties.Settings.Default.setTool_nudSetOfset = nudSetOfset;
            }
        }

        private void btnAddnewSlope_Click(object sender, EventArgs e)
        {
            if (mf.isSlopeline)
                CreatenewSlopeline();
        }

        private void btnleaveMenue_Click(object sender, EventArgs e)
        {
            /*  if (MessageBox.Show("Close without save tracks?", "  Are you sure? ",
                  MessageBoxButtons.YesNo) == DialogResult.Yes)
              {
                  Properties.Settings.Default.setTool_ToolLookahead = ToolLookahead;
                  Properties.Settings.Default.setTool_ToolPWM = ToolpWM;
                  Properties.Settings.Default.setTool_nudSetOfset = nudSetOfset;
                  Properties.Settings.Default.setTool_nudSetToolAnthight = nudSetToolAnthight;
                  mf.tool.isToolAntenna3 = false;
             */
            Close();
            // }
        }

        private void btnSlopeDirection_Click(object sender, EventArgs e)
        {
            if (!isSlopeDirectionLeft)
            {
                btnSlopeDirection.Image = AgOpenGPS.Properties.Resources.ArrowLeft;
                isSlopeDirectionLeft = true;
            }
            else
            {
                btnSlopeDirection.Image = AgOpenGPS.Properties.Resources.ArrowRight;
                isSlopeDirectionLeft = false;
            }
            //Console.WriteLine("isSlopeDirectionLeft " + isSlopeDirectionLeft.ToString());

        }

        private void hsbarToolLookAhead_ValueChanged(object sender, EventArgs e)
        {
            ToolLookahead = hsbarToolLookAhead.Value;
            lblToolLookAhead.Text = (hsbarToolLookAhead.Value).ToString();
            Properties.Settings.Default.setTool_ToolLookahead = ToolLookahead;
        }

        private void nudToolbehindPivot_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            nudSetToolPivot = (double)nudToolbehindPivot.Value;

        }

        private void hToolPWM_ValueChanged(object sender, EventArgs e)
        {
            ToolpWM = hToolPWM.Value;
            lblToolPWM.Text = (hToolPWM.Value).ToString();
        }

        private void rbtnToolAntennaLeft_CheckedChanged(object sender, EventArgs e)
        {
            if (rbtnToolAntennaRight.Checked)
            {
                nudSetOfset = (double)nudSetToolOffset.Value * -1;// mf.inchOrCm2m;
                rbtnToolAntennaLeft.Checked = false;
                rbtnToolAntennaRight.Checked = true;
                rbtnToolAntennaCenter.Checked = false;

            }
            else if (rbtnToolAntennaLeft.Checked)
            {
                nudSetOfset = (double)nudSetToolOffset.Value;// * mf.inchOrCm2m;
                rbtnToolAntennaLeft.Checked = true;
                rbtnToolAntennaRight.Checked = false;
                rbtnToolAntennaCenter.Checked = false;
            }
            else
            {
                rbtnToolAntennaLeft.Checked = false;
                rbtnToolAntennaRight.Checked = false;
                rbtnToolAntennaCenter.Checked = true;
                nudSetOfset = 0;
                nudSetToolOffset.Value = 0;
            }

            Properties.Settings.Default.setTool_nudSetOfset = nudSetOfset;

        }

        private void btnSetupopenclose_Click(object sender, EventArgs e)
        {
            btnSetupopenclose1_Click();
        }

        private void btnSetupopenclose1_Click()
        {
            if (isScreenbig)
            {
                btnSetupopenclose.Image = AgOpenGPS.Properties.Resources.ArrowRight;
                this.Size = new Size(110, 550);
                isScreenbig = false;
            }
            else
            {
                this.Size = new Size(500, 550);
                btnSetupopenclose.Image = AgOpenGPS.Properties.Resources.ArrowLeft;
                isScreenbig = true;
            }
        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            RollToolVeh = mf.ahrs.imuRoll;
            Calculate_All_Lines();

            if ((mf.isBtnAutoSteerOn) && (!mf.isSlopeline))
            {
                //mf.curve.DrawToolCurve();

                if (mf.tooltrk.isbtnAddToolTrackPts)
                {
                    AddToolTrackPts();
                }
                if (mf.tooltrk.isbtnToolAtWork)
                {
                    ToolAtWork();
                }
            }

            if ((mf.tooltrk.gToolArr.Count > 0) && (!mf.isThirdAntenne) && (mf.isBtnAutoSteerOn))
            {
                CreateAddRollinline();
                mf.curve.DrawSlope();
            }
        }
    }
}
