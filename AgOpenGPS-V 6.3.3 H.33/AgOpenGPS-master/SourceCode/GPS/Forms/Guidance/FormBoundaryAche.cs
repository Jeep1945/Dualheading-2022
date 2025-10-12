using AgOpenGPS.Classes;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;

namespace AgOpenGPS.Forms.Guidance
{
    public partial class FormBoundaryAche : Form
    {
        private readonly FormGPS mf = null;

        private Point fixPt;

        private vec3 ptA = new vec3();
        private vec3 ptB = new vec3();

        private bool isA = true, isA3p = true;
        private bool isC = false, isC3p = false;
        private int le_do_half = 0;
        private int start = 99999, end = 99999;
        private int start3p = 99999, end3p = 99999;
        private int startABA = 99999, endABA = 99999;
        private int startABB = 99999, endABB = 99999;
        private int startendABA = 0;
        private int startendABB = 0;
        private int bndSelect = 0;
        private int bndSelect3p = 0;
        private int bndSelectTrim = 0;
        public int LineSelect = 0;
        public int LineSelectA = 0;
        public int LineSelectB = 0;
        public int[,] Corner =
        {
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
        };
        private double dist1 = 0, dist2 = 0;
        private double distAway;
        public double[] nudgeLineBoundary = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        private double zoom = 1, sX = 0, sY = 0;
        public double CrossingpointEast, CrossingpointNorth;
        public double CrossingpointEast1, CrossingpointNorth1;
        public double CrossingpointEast2, CrossingpointNorth2;

        public vec3 pint = new vec3(0.0, 1.0, 0.0);

        private int indx = 1;

        public List<CTrk> gTemp = new List<CTrk>();

        public List<vec3> secList = new List<vec3>();
        public List<vec3> bndList = new List<vec3>();
        public List<vec3> fenceLine3p = new List<vec3>();
        public List<vec3> smooList = new List<vec3>();
        public List<vec3> tempList1 = new List<vec3>();
        public List<vec3> tempList2 = new List<vec3>();
        public List<vec3> tempListpart1 = new List<vec3>();
        public List<vec3> tempListpart2 = new List<vec3>();
        public List<vec3> tempListpart3 = new List<vec3>();
        public List<vec3> tempListpart4 = new List<vec3>();
        public List<vec3> tempListpart5 = new List<vec3>();
        public List<vec3> tempListpart6 = new List<vec3>();
        public List<vec3> backupList = new List<vec3>();
        public List<vec3> backupListABLine = new List<vec3>();
        public List<vec3> backupListABCurve = new List<vec3>();
        public List<vec3> backupListFence = new List<vec3>();
        public List<vec3> backupListFenceA = new List<vec3>();
        public List<List<vec3>> HelpList = new List<List<vec3>>();
        public List<List<vec3>> HelpListTrim = new List<List<vec3>>();
        public List<vec3> CornerPoints = new List<vec3>();
        public List<List<vec3>> TouchABPoints = new List<List<vec3>>();

        private bool isLinesVisible = true;

        public FormBoundaryAche(Form callingFormyt)
        {
            //get copy of the calling main form
            mf = callingFormyt as FormGPS;

            InitializeComponent();
            mf.CalculateMinMax();
            CalculateFenceArea();
            btnSlice.Checked = false;
            mf.bndl.tracksArrbndl.Clear();

            btnMoveUp.Visible = false;
            btnMoveDn.Visible = false;
            btnMoveLeft.Visible = false;
            btnMoveRight.Visible = false;
            btn_Area1.Visible = false;
            btn_Area2.Visible = false;
            btn_Area1.BackColor = Color.LightPink;
            btn_Area2.BackColor = Color.LightPink;
            label2.Visible = false;
            label3.Visible = false;
            lbldistance1.Visible = false;
            lbldistance2.Visible = false;
            btnCycleBackwardFirstAB.Visible = false;
            btnCycleForwardFirstAB.Visible = false;
            btnBndLoopbndl.Visible = false;


            //btnSlice_line.Checked = true;
        }

        private void FormBoundaryAch_Load(object sender, EventArgs e)
        {
            gTemp.Clear();

            foreach (var item in mf.trk.gArr)
            {
                gTemp.Add(new CTrk(item));
            }
            mf.Cont.idxCont = -1;

            //FixLabelsCurve();

            if (gTemp.Count != 0)
            {
                if (mf.trk.idx > -1 && mf.trk.idx <= gTemp.Count)
                {
                    indx = mf.trk.idx;
                }
                else
                    indx = 0;
            }

            mf.bndl.idxbndl = -1;

            backupListFence.Clear();
            backupListFenceA.Clear();

            foreach (var item in mf.bnd.bndList[0].fenceLine)
            {
                backupListFence.Add(item);
                backupListFenceA.Add(item);
            }


            lblToolWidth.Text = " Tool : "
                + ((mf.tool.width - mf.tool.overlap) * mf.m2FtOrM).ToString("N1") + mf.unitsFtM + " ";

            Size = Properties.Settings.Default.setWindow_BoundaryAchSize;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
            FormBoundaryAche_ResizeEnd(this, e);

            if (!mf.IsOnScreen(Location, Size, 1))
            {
                Top = 0;
                Left = 0;
            }
        }

        private void FormBoundaryLine_FormClosing(object sender, FormClosingEventArgs e)
        {
            //mf.FileSaveBoundary();

            if (mf.bndl.tracksArrbndl.Count > 0)
            {
                mf.bndl.idxbndl = 0;
            }
            else mf.bndl.idxbndl = -1;

            Properties.Settings.Default.setWindow_BoundaryAchSize = Size;
            Properties.Settings.Default.Save();
        }

        private void FormBoundaryAche_ResizeEnd(object sender, EventArgs e)
        {
            Width = (Height * 4 / 3);

            oglSelfbndl.Height = oglSelfbndl.Width = Height - 50;

            oglSelfbndl.Left = 2;
            oglSelfbndl.Top = 2;

            oglSelfbndl.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfbndl.Width, oglSelfbndl.Height);
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);

            tlp1bndl.Width = Width - oglSelfbndl.Width - 4;
            tlp1bndl.Left = oglSelfbndl.Width;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
        }

        private void oglSelfbndl_MouseDown(object sender, MouseEventArgs e)
        {
            if (!btnSlice.Checked)
            {
                if (cboxABBoundary.Checked)
                    oglSelfbndl_MouseDown3();
                else
                    oglSelfbndl_MouseDown1();
            }
            else
                oglSelfbndl_MouseDown2();
        }

        private void oglSelfbndl_MouseDown1() // all point cut with Lines from boundar 
        {
            Point pt = oglSelfbndl.PointToClient(Cursor.Position);

            int wid = oglSelfbndl.Width;
            int halfWid = oglSelfbndl.Width / 2;
            double scale = (double)wid * 0.903;
            /*
                        if (cboxIsZoombndl.Checked && !zoomTogglebndl)
                        {
                            sX = ((halfWid - (double)pt.X) / wid) * 1.1;
                            sY = ((halfWid - (double)pt.Y) / -wid) * 1.1;
                            zoom = 0.1;
                            zoomTogglebndl = true;
                            return;
                        }
            */

            //Convert to Origin in the center of window, 800 pixels
            fixPt.X = pt.X - halfWid;
            fixPt.Y = (wid - pt.Y - halfWid);

            vec3 plotPt = new vec3
            {
                //convert screen coordinates to field coordinates
                easting = fixPt.X * mf.maxFieldDistance / scale * zoom,
                northing = fixPt.Y * mf.maxFieldDistance / scale * zoom,
                heading = 0
            };

            plotPt.easting += mf.fieldCenterX + mf.maxFieldDistance * -sX;
            plotPt.northing += mf.fieldCenterY + mf.maxFieldDistance * -sY;

            pint.easting = plotPt.easting;
            pint.northing = plotPt.northing;

            if (mf.bndl.idxbndl < 1) btnSlice_line.Checked = true;
            else btnSlice_line.Checked = false;

            zoom = 1;
            sX = 0;
            sY = 0;

            if (mf.bnd.bndList.Count != 0)
            {
                if (start != 99999 & end != 99999)
                {
                    isC = true;
                    return;
                }
            }

            if (isA)
            {
                double minDistA = double.MaxValue;
                start = 99999; end = 99999;
                if (mf.bnd.bndList.Count != 0)
                {
                    for (int j = 0; j < mf.bnd.bndList.Count; j++)
                    {
                        for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                        {
                            double dist = ((pint.easting - mf.bnd.bndList[j].fenceLine[i].easting) * (pint.easting - mf.bnd.bndList[j].fenceLine[i].easting))
                                            + ((pint.northing - mf.bnd.bndList[j].fenceLine[i].northing) * (pint.northing - mf.bnd.bndList[j].fenceLine[i].northing));
                            if (dist < minDistA)
                            {
                                minDistA = dist;
                                bndSelect = j;
                                start = i;
                            }
                        }
                    }
                }
                isA = false;
            }
            else
            {
                double minDistA = double.MaxValue;
                int j = bndSelect;

                for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                {
                    double dist = ((pint.easting - mf.bnd.bndList[j].fenceLine[i].easting) * (pint.easting - mf.bnd.bndList[j].fenceLine[i].easting))
                                    + ((pint.northing - mf.bnd.bndList[j].fenceLine[i].northing) * (pint.northing - mf.bnd.bndList[j].fenceLine[i].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        end = i;
                    }
                }

                isA = true;

                if (start == end)
                {
                    start = 99999; end = 99999;
                    mf.TimedMessageBox(2000, "Line Error", "Start Point = End Point ");
                    return;
                }

                //build the lines
                if (rbtnCurvebndl.Checked)
                {

                    mf.bndl.tracksArrbndl.Add(new CBoundaryPath());
                    mf.bndl.idxbndl = mf.bndl.tracksArrbndl.Count - 1;

                    bool isLoop = false;
                    int limit = end;

                    if ((Math.Abs(start - end)) > (mf.bnd.bndList[bndSelect].fenceLine.Count * 0.5))
                    {
                        if (start < end)
                        {
                            (start, end) = (end, start);
                        }

                        isLoop = true;
                        if (start < end)
                        {
                            limit = end;
                            end = 0;
                        }
                        else
                        {
                            limit = end;
                            end = mf.bnd.bndList[bndSelect].fenceLine.Count;
                        }
                    }
                    else
                    {
                        if (start > end)
                        {
                            (start, end) = (end, start);
                        }
                    }

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].a_point = start;
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl?.Clear();

                    if (start < end)
                    {
                        for (int i = start; i <= end; i++)
                        {
                            //calculate the point inside the boundary
                            mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(new vec3(mf.bnd.bndList[bndSelect].fenceLine[i]));

                            if (isLoop && i == mf.bnd.bndList[bndSelect].fenceLine.Count - 1)
                            {
                                i = -1;
                                isLoop = false;
                                end = limit;
                            }
                        }
                    }
                    else
                    {
                        for (int i = start; i >= end; i--)
                        {
                            //calculate the point inside the boundary                            
                            mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(new vec3(mf.bnd.bndList[bndSelect].fenceLine[i]));

                            if (isLoop && i == 0)
                            {
                                i = mf.bnd.bndList[bndSelect].fenceLine.Count - 1;
                                isLoop = false;
                                end = limit;
                            }
                        }
                    }

                    //who knows which way it actually goes
                    mf.curve.CalculateHeadings(ref mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl);

                    int ptCnt = mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 1;

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[ptCnt]);
                        pnt.easting += (Math.Sin(pnt.heading) * i);
                        pnt.northing += (Math.Cos(pnt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(pnt);
                    }

                    vec3 stat = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0]);

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(stat);
                        pnt.easting -= (Math.Sin(pnt.heading) * i);
                        pnt.northing -= (Math.Cos(pnt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(0, pnt);
                    }

                    //create a name
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].name = mf.bndl.idxbndl.ToString() + " Cu " + DateTime.Now.ToString("mm:ss", CultureInfo.InvariantCulture);

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].moveDistance = 0;

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].mode = (int)TrackMode.Curve;

                    mf.FileSaveBoundary();

                    //update the arrays
                    start = 99999; end = 99999;

                    btnExitbndl.Focus();
                }
                else if (rbtnLinebndl.Checked)
                {
                    if ((Math.Abs(start - end)) > (mf.bnd.bndList[bndSelect].fenceLine.Count * 0.5))
                    {
                        if (start < end)
                        {
                            (start, end) = (end, start);
                        }
                    }
                    else
                    {
                        if (start > end)
                        {
                            (start, end) = (end, start);
                        }
                    }

                    vec3 ptA = new vec3(mf.bnd.bndList[bndSelect].fenceLine[start]);
                    vec3 ptB = new vec3(mf.bnd.bndList[bndSelect].fenceLine[end]);

                    //calculate the AB Heading
                    double abHead = Math.Atan2(
                        mf.bnd.bndList[bndSelect].fenceLine[end].easting - mf.bnd.bndList[bndSelect].fenceLine[start].easting,
                        mf.bnd.bndList[bndSelect].fenceLine[end].northing - mf.bnd.bndList[bndSelect].fenceLine[start].northing);
                    if (abHead < 0) abHead += glm.twoPI;

                    if (mf.bndl.idxbndl < mf.bndl.tracksArrbndl.Count - 1)
                    {
                        mf.bndl.idxbndl++;
                        mf.bndl.tracksArrbndl.Insert(mf.bndl.idxbndl, new CBoundaryPath());
                    }
                    else
                    {
                        mf.bndl.tracksArrbndl.Add(new CBoundaryPath());
                        mf.bndl.idxbndl = mf.bndl.tracksArrbndl.Count - 1;
                    }

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].a_point = start;
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl?.Clear();

                    ptA.heading = abHead;
                    ptB.heading = abHead;

                    for (int i = 0; i <= (int)(glm.Distance(ptA, ptB)); i++)
                    {
                        vec3 ptC = new vec3(ptA)
                        {
                            easting = (Math.Sin(abHead) * i) + ptA.easting,
                            northing = (Math.Cos(abHead) * i) + ptA.northing,
                            heading = abHead
                        };
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(ptC);
                    }

                    int ptCnt = mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 1;

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[ptCnt]);
                        pnt.easting += (Math.Sin(pnt.heading) * i);
                        pnt.northing += (Math.Cos(pnt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(pnt);
                    }

                    vec3 stat = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0]);

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(stat);
                        pnt.easting -= (Math.Sin(pnt.heading) * i);
                        pnt.northing -= (Math.Cos(pnt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(0, pnt);
                    }

                    //create a name
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].name = mf.bndl.idxbndl.ToString() + " AB " + DateTime.Now.ToString("hh:mm:ss", CultureInfo.InvariantCulture);

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].moveDistance = 0;

                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].mode = (int)TrackMode.AB;

                    mf.FileSaveBoundary();

                    start = 99999; end = 99999;
                }

                mf.bndl.desListbndl?.Clear();

                if (mf.bndl.tracksArrbndl.Count < 1 || mf.bndl.idxbndl == -1) return;

                distAway = (double)nudSetDistancebndl.Value * mf.ftOrMtoM;
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].moveDistance += distAway;

                double distSqAway = (distAway * distAway) - 0.01;
                vec3 point;

                int refCount = mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count;
                for (int i = 0; i < refCount; i++)
                {
                    point = new vec3(
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].easting - (Math.Sin(glm.PIBy2 + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].heading) * distAway),
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].northing - (Math.Cos(glm.PIBy2 + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].heading) * distAway),
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].heading);
                    bool Add = true;

                    for (int t = 0; t < refCount; t++)
                    {
                        double dist = ((point.easting - mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[t].easting) * (point.easting - mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[t].easting))
                            + ((point.northing - mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[t].northing) * (point.northing - mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[t].northing));
                        if (dist < distSqAway)
                        {
                            Add = false;
                            break;
                        }
                    }

                    if (Add)
                    {
                        if (mf.bndl.desListbndl.Count > 0)
                        {
                            double dist = ((point.easting - mf.bndl.desListbndl[mf.bndl.desListbndl.Count - 1].easting) * (point.easting - mf.bndl.desListbndl[mf.bndl.desListbndl.Count - 1].easting))
                                + ((point.northing - mf.bndl.desListbndl[mf.bndl.desListbndl.Count - 1].northing) * (point.northing - mf.bndl.desListbndl[mf.bndl.desListbndl.Count - 1].northing));
                            if (dist > 1)
                                mf.bndl.desListbndl.Add(point);
                        }
                        else mf.bndl.desListbndl.Add(point);
                    }
                }

                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();

                for (int i = 0; i < mf.bndl.desListbndl.Count; i++)
                {
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(new vec3(mf.bndl.desListbndl[i]));
                }

                if (mf.bndl.idxbndl == 0) btnSlice_line.Checked = true;
                else btnSlice_line.Checked = false;

                mf.CalculateMinMax();
                mf.bndl.desListbndl?.Clear();
            }
        }

        private void oglSelfbndl_MouseDown2()   // for 3 point cut 
        {
            Point ptt = oglSelfbndl.PointToClient(Cursor.Position);

            int wid = oglSelfbndl.Width;
            int halfWid = oglSelfbndl.Width / 2;
            double scale = (double)wid * 0.903;
            /*
                        if (cboxIsZoombndl.Checked)
                        {
                            sX = ((halfWid - (double)ptt.X) / wid) * 1.1;
                            sY = ((halfWid - (double)ptt.Y) / -wid) * 1.1;
                            zoom = 0.1;
                            cboxIsZoombndl.Checked = false;
                            return;
                        }
            */
            //if (mf.bnd.bndList.Count < 1) { return; }

            //Convert to Origin in the center of window, 800 pixels
            fixPt.X = ptt.X - halfWid;
            fixPt.Y = (wid - ptt.Y - halfWid);
            vec3 plotPt = new vec3
            {
                //convert screen coordinates to field coordinates
                easting = fixPt.X * mf.maxFieldDistance / scale * zoom,
                northing = fixPt.Y * mf.maxFieldDistance / scale * zoom,
                heading = 0
            };

            plotPt.easting += mf.fieldCenterX + mf.maxFieldDistance * -sX;
            plotPt.northing += mf.fieldCenterY + mf.maxFieldDistance * -sY;

            pint.easting = plotPt.easting;
            pint.northing = plotPt.northing;

            if (mf.bnd.bndList.Count != 0)
            {
                if (start3p != 99999 & end3p != 99999)
                {
                    isC3p = true;

                    btnMoveUp.Visible = true;
                    btnMoveDn.Visible = true;
                    btnMoveLeft.Visible = true;
                    btnMoveRight.Visible = true;

                    //return;
                    goto DistancePoints;
                }
            }

            if (isA3p)
            {
                double minDistA = double.MaxValue;
                start3p = 99999; end3p = 99999;
                if (mf.bnd.bndList.Count != 0)
                {
                    for (int j = 0; j < mf.bnd.bndList.Count; j++)
                    {
                        for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                        {
                            double dist = ((pint.easting - mf.bnd.bndList[j].fenceLine[i].easting) * (pint.easting - mf.bnd.bndList[j].fenceLine[i].easting))
                                            + ((pint.northing - mf.bnd.bndList[j].fenceLine[i].northing) * (pint.northing - mf.bnd.bndList[j].fenceLine[i].northing));
                            if (dist < minDistA)
                            {
                                minDistA = dist;
                                bndSelect3p = j;
                                start3p = i;
                            }
                        }
                    }
                }
                else
                {
                    start3p = 1;
                    ptA = pint;
                }

                isA3p = false;
            }
            else
            {
                double minDistA = double.MaxValue;
                int j = bndSelect3p;

                if (mf.bnd.bndList.Count != 0)
                {

                    for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                    {
                        double dist = ((pint.easting - mf.bnd.bndList[j].fenceLine[i].easting) * (pint.easting - mf.bnd.bndList[j].fenceLine[i].easting))
                                        + ((pint.northing - mf.bnd.bndList[j].fenceLine[i].northing) * (pint.northing - mf.bnd.bndList[j].fenceLine[i].northing));
                        if (dist < minDistA)
                        {
                            minDistA = dist;
                            end3p = i;
                        }
                    }
                }
                else
                {
                    end3p = 1;
                    ptB = pint;
                }
                btnBndLoopbndl.Visible = true;
                btnBndLoopbndl.BackColor = Color.SandyBrown;

                isA3p = true;
            }

            DistancePoints:

            if (start3p != 99999)
            {
                dist1 = glm.Distance(mf.bnd.bndList[bndSelect3p].fenceLine[start3p], pint);
                lbldistance1.Visible = true;
                lbldistance1.Text = Convert.ToString(dist1);
            }
            else
            {
                //dist1 = 0;
                //lbldistance1.Visible = false;
                //lbldistance1.Text = Convert.ToString(dist1);
            }

            if (end3p != 99999)
            {
                dist2 = glm.Distance(mf.bnd.bndList[bndSelect3p].fenceLine[end3p], pint);
                lbldistance2.Visible = true;
                lbldistance2.Text = Convert.ToString(dist2);
            }
            else
            {
                //dist2 = 0;
                //lbldistance2.Visible = false;
                //lbldistance2.Text = Convert.ToString(dist2);
            }

        }

        private void DrawABTouchPointsABBoundary()   // draw the point for AB boundary 
        {

            //for (int ikx = 0; ikx < HelpList.Count - 1; ikx++)
            //{
            if (TouchABPoints.Count >= 0)
            {
                for (int ikx = 0; ikx < TouchABPoints.Count; ikx++)
                {
                    if (TouchABPoints[ikx].Count > 1)
                    {
                        if (TouchABPoints[ikx].Count == 2)    // touchlines
                        {
                            GL.PointSize(1);
                            GL.Color3(0.30f, 0.7f, 0.30f);
                            GL.Begin(PrimitiveType.Lines);
                            if (bndSelectTrim == ikx)
                                GL.Color3(0.30f, 0.7f, 1.30f);
                            GL.Vertex3(TouchABPoints[ikx][0].easting, TouchABPoints[ikx][0].northing, 0);
                            GL.Vertex3(TouchABPoints[ikx][1].easting, TouchABPoints[ikx][1].northing, 0);
                            GL.End();
                        }
                        if (TouchABPoints[ikx].Count != 0)
                        { // first point  
                            GL.PointSize(16);
                            GL.Begin(PrimitiveType.Points);
                            GL.Color3(0, 0, 0);  //black of background point 1 and 2
                            GL.Vertex3(TouchABPoints[ikx][0].easting, TouchABPoints[ikx][0].northing, 0);
                            GL.End();
                            GL.PointSize(6);
                            GL.Begin(PrimitiveType.Points);
                            GL.Color3(1.0f, 0.75f, 0.350f); //  Point A beige
                            GL.Vertex3(TouchABPoints[ikx][0].easting, TouchABPoints[ikx][0].northing, 0);
                            GL.End();
                        }
                        if (TouchABPoints[ikx].Count != 1)    // second point
                        {
                            GL.PointSize(16);
                            GL.Begin(PrimitiveType.Points);
                            GL.Color3(0, 0, 0);  //black of background point 1 and 2
                            GL.Vertex3(TouchABPoints[ikx][TouchABPoints[ikx].Count - 1].easting, TouchABPoints[ikx][TouchABPoints[ikx].Count - 1].northing, 0);
                            GL.End();
                            GL.PointSize(6);
                            GL.Begin(PrimitiveType.Points);
                            //GL.Color3(1f, 0.2f, 0.2f);
                            GL.Color3(0.5f, 0.75f, 1.0f);
                            GL.Vertex3(TouchABPoints[ikx][TouchABPoints[ikx].Count - 1].easting, TouchABPoints[ikx][TouchABPoints[ikx].Count - 1].northing, 0);
                            GL.End();
                        }
                        if (CornerPoints.Count > ikx)
                        {
                            double Texthight, Texthightvalue = 1;
                            Texthight = Texthightvalue / 10;
                            GL.PointSize(5.0f);
                            GL.Begin(PrimitiveType.Points);
                            GL.Color3(0.95f, 0.90f, 0.0f);  // yellow
                            GL.Vertex3(CornerPoints[ikx].easting, CornerPoints[ikx].northing, 0.0);
                            GL.End();
                            string textA1 = Convert.ToString(ikx);
                            GL.Enable(EnableCap.Texture2D);
                            GL.BindTexture(TextureTarget.Texture2D, mf.texture[(int)FormGPS.textures.ZoomIn48]);        // Select Our Texture
                            mf.font.DrawText(CornerPoints[ikx].easting + Texthightvalue, CornerPoints[ikx].northing + Texthightvalue, textA1, Texthight);
                            GL.End();
                        }
                    }
                }/*
                for (int iCorner = 0; iCorner < CornerPoints.Count - 1; iCorner++)
                {
                    GL.PointSize(2);
                    GL.Begin(PrimitiveType.Lines);
                    GL.Color3(0.30f, 0.7f, 1.30f);
                    GL.Vertex3(CornerPoints[iCorner].easting, CornerPoints[iCorner].northing, 0);
                    GL.Vertex3(CornerPoints[iCorner + 1].easting, CornerPoints[iCorner + 1].northing, 0);
                    //GL.Vertex3(CornerPoints[CornerPoints.Count - 1].easting, CornerPoints[CornerPoints.Count - 1].northing, 0);
                    L.End();
                }
                //GL.End();
                //GL.Color3(2.0f, 0.20f, 0.20f);      // red
                // GL.Color3(0.30f, 0.7f, 1.30f);      // green
                */
            }
            GL.End();
        }


        private void MakeABLineCurveRealBorder()  // makes boundary line & curves for AB boundary 

        {
            int starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int isStart = 0;
            int ik;
            ik = indx;

            if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > -1)
            {
                mf.bnd.bndList[bndSelect].fenceLine_new.Clear();
                foreach (var item in mf.bnd.bndList[bndSelect].fenceLine)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                }
            }
            //save a backup fenceline
            backupList?.Clear();
            foreach (var item in mf.bnd.bndList[bndSelect].fenceLine_new)
            {
                backupList.Add(item);
            }
           
            mf.bndl.tracksArrbndl?.Clear();
            /*
                        //if (TouchABPoints[ikx].Count > 0)
                        Console.WriteLine("  Start");
                        Console.WriteLine("Touch : " + 11 + " " + HelpListTrim[0][0].easting + " : " + TouchABPoints[0][0].northing);
                        Console.WriteLine("Touch : " + 12 + " " + HelpListTrim[0][1].easting + " : " + TouchABPoints[0][1].northing);
                        Console.WriteLine("Touch : " + 21 + " " + HelpListTrim[1][0].easting + " : " + TouchABPoints[1][0].northing);
                        Console.WriteLine("Touch : " + 22 + " " + HelpListTrim[1][1].easting + " : " + TouchABPoints[1][1].northing);
                        Console.WriteLine("Touch : " + 31 + " " + HelpListTrim[2][0].easting + " : " + TouchABPoints[2][0].northing);
                        Console.WriteLine("Touch : " + 32 + " " + HelpListTrim[2][1].easting + " : " + TouchABPoints[2][1].northing);
                        Console.WriteLine("Touch : " + 41 + " " + HelpListTrim[3][0].easting + " : " + TouchABPoints[3][0].northing);
                        Console.WriteLine("Touch : " + 42 + " " + HelpListTrim[3][1].easting + " : " + TouchABPoints[3][1].northing);
            */
            //sort the lines
            //mf.bndl.tracksArrbndl.Sort((p, q) => p.a_point.CompareTo(q.a_point));
            //mf.FileSaveBoundary();

            mf.bndl.idxbndl = -1;

            int numOfLines = HelpListTrim.Count;
            int nextLine = 0;
            crossingsbndl.Clear();


            for (int lineNum = 0; lineNum < HelpListTrim.Count; lineNum++)
            {
                nextLine = lineNum - 1;
                if (nextLine < 0) nextLine = HelpListTrim.Count - 1;

                if (nextLine == lineNum)
                {
                    mf.TimedMessageBox(2000, "Create Error", "Is there maybe only 1 line?");
                    return;
                }
                for (int i = 0; i < HelpListTrim[lineNum].Count - 2; i++)
                {
                    for (int k = 0; k < HelpListTrim[nextLine].Count - 2; k++)
                    {
                        int res = GetLineIntersectionbndl(
                        HelpListTrim[lineNum][i].easting,
                        HelpListTrim[lineNum][i].northing,
                        HelpListTrim[lineNum][i + 1].easting,
                        HelpListTrim[lineNum][i + 1].northing,

                        HelpListTrim[nextLine][k].easting,
                        HelpListTrim[nextLine][k].northing,
                        HelpListTrim[nextLine][k + 1].easting,
                        HelpListTrim[nextLine][k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0)
                            {
                                starttrack = i + 1;
                                startbndl = k;
                                // insert crossing startpoint in cutline
                                CrossingpointEast1 = CrossingpointEast;
                                CrossingpointNorth1 = CrossingpointNorth;
                                vec3 t1 = new vec3(CrossingpointEast1, CrossingpointNorth1, TouchABPoints[lineNum][i].heading);
                                vec3 t2 = new vec3(CrossingpointEast1, CrossingpointNorth1, TouchABPoints[nextLine][k].heading);
                                HelpListTrim[lineNum].Insert(starttrack, t1);
                                HelpListTrim[nextLine].Insert(startbndl, t2);
                            }
                            else if (isStart == 2)
                            {
                                endtrack = i + 1;
                                endbndl = k + 1;
                                if (CrossingpointEast != 0 && CrossingpointNorth != 0)
                                {
                                    CrossingpointEast2 = CrossingpointEast;
                                    CrossingpointNorth2 = CrossingpointNorth;
                                    vec3 t3 = new vec3(CrossingpointEast2, CrossingpointNorth2, TouchABPoints[lineNum][i].heading);
                                    vec3 t4 = new vec3(CrossingpointEast2, CrossingpointNorth2, TouchABPoints[nextLine][k].heading);
                                    HelpListTrim[lineNum].Insert(endtrack, t3);
                                    HelpListTrim[nextLine].Insert(endbndl, t4);

                                }
                            }
                            nextLine = lineNum + 1;

                            if (nextLine > mf.bndl.tracksArrbndl.Count - 1) nextLine = 0;

                            isStart++;
                        }
                    }

                }

            }
            int zu = TouchABPoints.Count;


            return;

            for (int l = 0; l < mf.bndl.tracksArrbndl[0].trackPtsbndl.Count; l++)
                Console.WriteLine("mf.bndl.tracksArrbndl 0 : " + l + " " + mf.bndl.tracksArrbndl[0].trackPtsbndl[l].easting + " : " + mf.bndl.tracksArrbndl[0].trackPtsbndl[l].northing);
            for (int l = 0; l < mf.bndl.tracksArrbndl[1].trackPtsbndl.Count; l++)
                Console.WriteLine("mf.bndl.tracksArrbndl 1 : " + l + " " + mf.bndl.tracksArrbndl[1].trackPtsbndl[l].easting + " : " + mf.bndl.tracksArrbndl[1].trackPtsbndl[l].northing);
            for (int l = 0; l < mf.bndl.tracksArrbndl[2].trackPtsbndl.Count; l++)
                Console.WriteLine("mf.bndl.tracksArrbndl 2 : " + l + " " + mf.bndl.tracksArrbndl[2].trackPtsbndl[l].easting + " : " + mf.bndl.tracksArrbndl[2].trackPtsbndl[l].northing);
            for (int l = 0; l < mf.bndl.tracksArrbndl[3].trackPtsbndl.Count; l++)
                Console.WriteLine("mf.bndl.tracksArrbndl 3 : " + l + " " + mf.bndl.tracksArrbndl[3].trackPtsbndl[l].easting + " : " + mf.bndl.tracksArrbndl[3].trackPtsbndl[l].northing);
            return;
            //                   Console.WriteLine("fenceLine_new 0 :  " + mf.bnd.bndList[bndSelect].fenceLine_new[startbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[startbndl].northing);
            //                   Console.WriteLine("fenceLine_new 1 :  " + mf.bnd.bndList[bndSelect].fenceLine_new[endbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[endbndl].northing);


            if (isStart < 2)
            {
                mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                mf.bndl.tracksArrbndl.Clear();
                return;
            }
            //}   // end of ABcurve  mode

            else if (gTemp[ik].mode == (int)TrackMode.AB)  // Line of choise from list
            {
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();
                int NewABLength = 200;

                vec3 PointsABLine = new vec3(0.0, 0.0, 0.0);
                vec3 PointsABLine1 = new vec3(0.0, 0.0, 0.0);
                PointsABLine.easting = gTemp[ik].ptA.easting - (Math.Sin(gTemp[ik].heading) * NewABLength) + Math.Cos(gTemp[ik].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM; // / 4 * 3;
                PointsABLine.northing = gTemp[ik].ptA.northing - (Math.Cos(gTemp[ik].heading) * NewABLength) - Math.Sin(gTemp[ik].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM; // / 4 * 3;
                PointsABLine.heading = gTemp[ik].heading;
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(PointsABLine);
                for (int iPoi = 1; iPoi < 150; iPoi++)
                {
                    PointsABLine1.easting = PointsABLine.easting + (Math.Sin(gTemp[ik].heading) * 25 * iPoi); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    PointsABLine1.northing = PointsABLine.northing + (Math.Cos(gTemp[ik].heading) * 25 * iPoi); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    PointsABLine1.heading = gTemp[ik].heading;
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(PointsABLine1);
                }

                tempListpart6.Clear();

                if (mf.bndl.idxbndl > -1)
                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count < 2) return;

                if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > -1)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Clear();
                    foreach (var item in mf.bnd.bndList[bndSelect].fenceLine)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                    }
                }

                //save a backup fenceline
                backupList?.Clear();
                foreach (var item in mf.bnd.bndList[bndSelect].fenceLine_new)
                {
                    backupList.Add(item);
                }

                vec3 pointcrosstrackstart = new vec3(0.0, 0.0, 0.0);
                tempListpart5.Clear();

                for (int i = 0; i < mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 2; i++)
                {
                    for (int k = 0; k < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 2; k++)
                    {
                        int res = GetLineIntersectionbndl(
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].northing,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].northing,

                        mf.bnd.bndList[0].fenceLine_new[k].easting,
                        mf.bnd.bndList[0].fenceLine_new[k].northing,
                        mf.bnd.bndList[0].fenceLine_new[k + 1].easting,
                        mf.bnd.bndList[0].fenceLine_new[k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0)
                            {
                                starttrack = i + 1;
                                startbndl = k + 1;

                            }
                            else
                            {
                                endtrack = i;
                                endbndl = k + 1;
                            }
                            isStart++;
                        }
                    }
                }

                if (isStart < 2)
                {
                    mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                    return;
                }


            }           // end of ABline  mode

            // start for all lines
            // start build off new fence line parts
            tempList1.Clear();
            tempList2.Clear();
            tempListpart5.Clear();

            for (int i = starttrack - 1; i < endtrack + 2; i++)
            {
                tempListpart5.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }


            // original direction of trackline 
            tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[startbndl]);
            for (int i = starttrack; i < endtrack; i++)
            {
                tempList1.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }
            // opposite direction of trackline 
            tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[endbndl]);
            tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[endbndl]);
            for (int i = endtrack; i > starttrack; i--)
            {
                tempList2.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }
            tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[startbndl]);

            if (startbndl > endbndl)
            {
                (startbndl, endbndl) = (endbndl, startbndl);
                (tempList1, tempList2) = (tempList2, tempList1);
            }

            // fence with Start&End
            // first fenceline_new segment is cutline part
            // tempList1
            // second segment
            for (int i = endbndl; i < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 1; i++)
            {
                tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }
            // dirth segment
            for (int i = 0; i < startbndl; i++)
            {
                tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }
            // fence without Start&End
            // first fenceline_new segment is cutline part
            // tempList2
            // second segment
            for (int i = startbndl; i < endbndl + 1; i++)
            {
                tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }

            mf.CalculateMinMax();
            CalculateFenceArea();
        }

        private void oglSelfbndl_MouseDown3()   // for Boundary with only AB boundary 
        {
            Point pt = oglSelfbndl.PointToClient(Cursor.Position);

            int starttrack = 0, endtrack = 0;
            int wid = oglSelfbndl.Width;
            int halfWid = oglSelfbndl.Width / 2;
            double scale = (double)wid * 0.903;
            /*
                        if (cboxIsZoombndl.Checked && !zoomTogglebndl)
                        {
                            sX = ((halfWid - (double)pt.X) / wid) * 1.1;
                            sY = ((halfWid - (double)pt.Y) / -wid) * 1.1;
                            zoom = 0.1;
                            zoomTogglebndl = true;
                            return;
                        }
            */

            //Convert to Origin in the center of window, 800 pixels
            fixPt.X = pt.X - halfWid;
            fixPt.Y = (wid - pt.Y - halfWid);

            vec3 plotPt = new vec3
            {
                //convert screen coordinates to field coordinates
                easting = fixPt.X * mf.maxFieldDistance / scale * zoom,
                northing = fixPt.Y * mf.maxFieldDistance / scale * zoom,
                heading = 0
            };

            plotPt.easting += mf.fieldCenterX + mf.maxFieldDistance * -sX;
            plotPt.northing += mf.fieldCenterY + mf.maxFieldDistance * -sY;

            pint.easting = plotPt.easting;
            pint.northing = plotPt.northing;

            zoom = 1;
            sX = 0;
            sY = 0;
            // touch Corner find first line 
            double minDistA = double.MaxValue;
            if (HelpList.Count > 2)
            {
                for (int j = 0; j < HelpList.Count; j++)
                {
                    for (int i = 0; i < HelpList[j].Count; i++)
                    {
                        double dist = ((pint.easting - HelpList[j][i].easting) * (pint.easting - HelpList[j][i].easting))
                                        + ((pint.northing - HelpList[j][i].northing) * (pint.northing - HelpList[j][i].northing));
                        if (dist < minDistA)
                        {
                            minDistA = dist;
                            LineSelectA = j;  // number of line in gTemp
                            startendABA = i;  // number of point in HelpList
                        }
                    }
                }
            }

            // touch Corner find second line 
            double minDistB = double.MaxValue;
            if (HelpList.Count > 2)
            {
                for (int j = 0; j < HelpList.Count; j++)
                {
                    if (j != LineSelectA)
                    {
                        for (int i = 0; i < HelpList[j].Count; i++)
                        {
                            double dist = ((pint.easting - HelpList[j][i].easting) * (pint.easting - HelpList[j][i].easting))
                                            + ((pint.northing - HelpList[j][i].northing) * (pint.northing - HelpList[j][i].northing));
                            if (dist < minDistB)
                            {
                                minDistB = dist;
                                LineSelectB = j;  // number of line in gTemp
                                startendABB = i;  // number of point in HelpList
                            }
                        }
                    }
                }
            }

            int startendABA1 = startendABA;
            startendABA1 += 3; if (startendABA1 > HelpList[LineSelectA].Count - 2) startendABA1 = HelpList[LineSelectA].Count - 2;
            int startendABB1 = startendABB;
            startendABB1 += 3; if (startendABB1 > HelpList[LineSelectB].Count - 2) startendABB1 = HelpList[LineSelectB].Count - 2;
            int startendABA2 = startendABA;
            startendABA2 -= 2; if (startendABA2 < 0) startendABA2 = 0;
            int startendABB2 = startendABB;
            startendABB2 -= 2; if (startendABB2 < 0) startendABB2 = 0;

            for (int i = startendABA - 2; i < startendABA1; i++)
            {
                for (int k = startendABB - 2; k < startendABB1; k++)
                {
                    int res = GetLineIntersectionbndl(
                    HelpList[LineSelectA][i].easting,
                    HelpList[LineSelectA][i].northing,
                    HelpList[LineSelectA][i + 1].easting,
                    HelpList[LineSelectA][i + 1].northing,

                    HelpList[LineSelectB][k].easting,
                    HelpList[LineSelectB][k].northing,
                    HelpList[LineSelectB][k + 1].easting,
                    HelpList[LineSelectB][k + 1].northing,
                    ref iE, ref iN);
                    if (res == 1)
                    {
                        starttrack = i + 1;
                        endtrack = k + 1;
                    }
                }
            }


            vec3 Corner1 = new vec3(HelpList[LineSelectA][startendABA].easting, HelpList[LineSelectA][startendABA].northing, HelpList[LineSelectA][startendABA].heading);
            vec3 Corner2 = new vec3(HelpList[LineSelectB][startendABB].easting, HelpList[LineSelectB][startendABB].northing, HelpList[LineSelectB][startendABB].heading);
            vec3 Corner3 = new vec3(HelpList[LineSelectB][startendABB].easting, HelpList[LineSelectB][startendABB].northing, 0);
            HelpList[LineSelectA].Insert(starttrack, Corner1);
            HelpList[LineSelectB].Insert(endtrack, Corner2);

            for (int x = 0; x < mf.bndl.tracksCorner.Count - 1; x++)
            {
                if ((mf.bndl.tracksCorner[x].LineA == LineSelectA) && (mf.bndl.tracksCorner[x].LineB == LineSelectB) && (mf.bndl.tracksCorner[x].exist))
                {
                    mf.bndl.tracksCorner?.Clear();
                    mf.bndl.tracksCorner[x].exist = false;
                }
                else
                {
                    if ((mf.bndl.tracksCorner[x].LineA == 0) || (mf.bndl.tracksCorner[x].LineB == 0) && (!mf.bndl.tracksCorner[x].exist))
                    {
                        //mf.bndl.tracksCorner[x].
                         //   mf.bndl.tracksCorner[x].PtsCorner.Add(Corner3);
                    }
                }
            }


            // put the choosen point to right line and position
            if (TouchABPoints[LineSelectA].Count == 2)
            {
                TouchABPoints[LineSelectA].Clear();
                TouchABPoints[LineSelectA].Add(Corner1);
                startABA = starttrack;
                goto FirstPoint;
            }

            if (TouchABPoints[LineSelectA].Count == 1)
            {
                TouchABPoints[LineSelectA].Add(Corner1);
                nudgeLineBoundary[LineSelectA] = (double)nudSetDistancebndl.Value;
                endABA = starttrack;
            }
            if (TouchABPoints[LineSelectA].Count == 0)
            {
                TouchABPoints[LineSelectA].Add(Corner1);
                startABA = starttrack;
            }

            if (TouchABPoints[LineSelectB].Count == 2)
            {
                TouchABPoints[LineSelectB].Clear();
                TouchABPoints[LineSelectB].Add(Corner2);
                startABB = endtrack;
                goto FirstPoint;
            }

            if (TouchABPoints[LineSelectB].Count == 1)
            {
                TouchABPoints[LineSelectB].Add(Corner2);
                nudgeLineBoundary[LineSelectB] = (double)nudSetDistancebndl.Value;
                endABB = endtrack;
            }
            if (TouchABPoints[LineSelectB].Count == 0)
            {
                TouchABPoints[LineSelectB].Add(Corner2);
                startABB = endtrack;
            }
            FirstPoint:

            double AngleHeading = 0;
            /*            if (TouchABPoints[LineSelect].Count == 2)
                        {// second point
                            if (startABA > endABA)
                            {
                                (startABA, endABA) = (endABA, startABA);
                                (TouchABPoints[LineSelect][0], TouchABPoints[LineSelect][1]) = (TouchABPoints[LineSelect][1], TouchABPoints[LineSelect][0]);
                                vec3 HelpABLine = new vec3(TouchABPoints[LineSelect][0]);
                                HelpABLine.heading += Math.PI;
                                TouchABPoints[LineSelect].RemoveAt(0);
                                TouchABPoints[LineSelect].Insert(0, HelpABLine);
                                HelpABLine = new vec3(TouchABPoints[LineSelect][1]);
                                HelpABLine.heading += Math.PI;
                                TouchABPoints[LineSelect].RemoveAt(1);
                                TouchABPoints[LineSelect].Add(HelpABLine);
                            }
                        }
                        */

            if (TouchABPoints[LineSelect].Count == 2)
            {
                HelpListTrim[LineSelect].Clear();
                HelpListTrim[LineSelect].Add(TouchABPoints[LineSelect][0]);
                double HelpLength = glm.Distance(TouchABPoints[LineSelect][0], TouchABPoints[LineSelect][1]);
                vec3 HelpABLine = new vec3(TouchABPoints[LineSelect][0]);
                Console.WriteLine("HelpLength   : " + HelpLength);
                int Pointdiff = 20;
                AngleHeading = gTemp[LineSelect].heading;
                while (Pointdiff < HelpLength)
                {
                    Pointdiff += 20;
                    HelpABLine.easting += Math.Sin(AngleHeading) * 20 - Math.Sin(glm.PIBy2 + AngleHeading) * (double)nudSetDistancebndl.Value * -mf.m2FtOrM;
                    HelpABLine.northing += Math.Cos(AngleHeading) * 20 - Math.Cos(glm.PIBy2 + AngleHeading) * (double)nudSetDistancebndl.Value * -mf.m2FtOrM;
                    HelpABLine.heading = AngleHeading;
                    HelpListTrim[bndSelect].Add(HelpABLine);
                }
                Console.WriteLine("endAB   : " + TouchABPoints[LineSelect][1].easting + "  " + TouchABPoints[LineSelect][1].northing);

            }
        }

        private void oglSelfbndl_Paint(object sender, PaintEventArgs e)
        {
            oglSelfbndl.MakeCurrent();

            GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.ColorBufferBit);
            GL.LoadIdentity();                  // Reset The View

            //back the camera up
            GL.Translate(0, 0, -mf.maxFieldDistance * zoom);

            //translate to that spot in the world
            GL.Translate(-mf.fieldCenterX + sX * mf.maxFieldDistance, -mf.fieldCenterY + sY * mf.maxFieldDistance, 0);

            //draw all the original boundaries 
            GL.LineWidth(1);

            for (int j = 0; j < mf.bnd.bndList.Count; j++)  // draw original fenceline
            {
                GL.Color4(0.0f, 1.0f, 1.0f, 0.98);

                if (mf.bnd.bndList[j].fenceLine_original.Count > 2)
                { }
                else
                {
                    for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                        mf.bnd.bndList[j].fenceLine_original.Add(mf.bnd.bndList[j].fenceLine[i]);
                }

                if (mf.bnd.bndList[0].fenceLine.Count > 2)
                {
                    GL.Color4(0.0f, 1.0f, 0.0f, 0.98);
                    mf.bnd.bndList[0].fenceLine_original.DrawPolygon();   // neon green out fenceLine

                    GL.Color3(0.8f, 0.8f, 0.8f);
                    mf.bnd.bndList[j].fenceLine_original.DrawPolygon();            // white inner fenceLine
                }

                GL.Begin(PrimitiveType.Points);
                GL.End();

            }

            if (cboxABBoundary.Checked)
            {
                //HelpList.Add(new List<vec3>());
                DrawAllLinesA();
                DrawABTouchPointsABBoundary();
                //DrawABTouchPointsABBoundaryLines();
                //DrawABTouchPointsABBoundary();
                //the vehicle
                GL.PointSize(8.0f);
                GL.Begin(PrimitiveType.Points);
                GL.Color3(0.95f, 0.90f, 0.0f);
                GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);


                GL.End();
                GL.PointSize(4.0f);

                GL.Begin(PrimitiveType.Points);
                GL.Color3(0.00f, 0.0f, 0.0f);
                GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
                GL.End();


                oglSelfbndl.SwapBuffers();
                // goto goon;
                return;
            }

            if (btnSlice.Checked)
            {
                //draw the line building graphics
                if (start3p != 99999 || end3p != 99999) DrawABTouchPoints();

                if (mf.bnd.bndList.Count != 0)
                {
                    //draw the actual built  3 points slice lines
                    if (start3p != 99999 && end3p != 99999)
                    {
                        if (isC3p)
                        {
                            GL.LineWidth(2);
                            GL.Color3(0.90f, 0.5f, 0.25f);
                            GL.Begin(PrimitiveType.LineStrip);
                            {
                                GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[start3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[start3p].northing, 0);
                                GL.Vertex3(pint.easting, pint.northing, 0);
                                GL.Color3(0.00f, 0.5f, 0.25f);
                                GL.Vertex3(pint.easting, pint.northing, 0);
                                GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[end3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[end3p].northing, 0);
                            }
                            GL.End();
                        }
                        else
                        {
                            GL.LineWidth(2);
                            GL.Color3(0.90f, 0.5f, 0.25f);
                            GL.Begin(PrimitiveType.Lines);
                            {
                                GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[start3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[start3p].northing, 0);
                                GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[end3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[end3p].northing, 0);
                            }
                            GL.End();

                        }
                    }
                }
                DrawBuiltLines();
            }
            else
            {
                if (cboxABLineCurve.Checked)
                {
                    FixLabelsCurveA();
                    DrawBuiltLinesA();
                    DrawABTouchLine();
                }
                else
                {
                    DrawBuiltLines();
                    DrawABTouchLine();
                }
            }
            GL.Disable(EnableCap.Blend);

            // the Tool
            double sinHR = Math.Sin(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * mf.m2FtOrM * 0.5;
            double cosHR = Math.Cos(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * mf.m2FtOrM * 0.5;
            double sinHL = Math.Sin(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * mf.m2FtOrM * 0.5;
            double cosHL = Math.Cos(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * mf.m2FtOrM * 0.5;

            GL.Color3(0.90f, 0.5f, 0.25f);
            GL.Begin(PrimitiveType.Lines);
            {
                GL.Vertex3(mf.pivotAxlePos.easting - sinHR, mf.pivotAxlePos.northing - cosHR, 0);
                GL.Vertex3(mf.pivotAxlePos.easting + sinHL, mf.pivotAxlePos.northing + cosHL, 0);
                GL.Vertex3(mf.pivotAxlePos.easting + sinHL, mf.pivotAxlePos.northing + cosHL, 0);
                GL.Vertex3(mf.pivotAxlePos.easting - sinHR, mf.pivotAxlePos.northing - cosHR, 0);
            }
            GL.End();

            //the vehicle
            GL.PointSize(8.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);


            GL.End();
            GL.PointSize(4.0f);

            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.00f, 0.0f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
            GL.End();


            GL.Flush();
            oglSelfbndl.SwapBuffers();

            int Valueidx = mf.bndl.idxbndl + 1;
            if (Valueidx > mf.bndl.tracksArrbndl.Count)
                Valueidx = 0;
            label4.Text = (Valueidx + " / " + mf.bndl.tracksArrbndl.Count);

            if (mf.bndl.tracksArrbndl.Count > 0) btnBndLoopbndl.Visible = true;
            else btnBndLoopbndl.Visible = false;
            if (btnSlice.Checked) btnBndLoopbndl.Visible = true;
        }

        private void oglSelfbndl_Resize(object sender, EventArgs e)
        {
            oglSelfbndl.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfbndl.Width, oglSelfbndl.Height);

            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);
        }

        private void DrawABTouchPoints()   // draw the 3 point slice touchpoints 
        {
            GL.PointSize(24);
            GL.Begin(PrimitiveType.Points);

            if (mf.bnd.bndList.Count != 0)
            {
                GL.Color3(0, 0, 0);  //black of background point 1 and 2
                if (start3p != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[start3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[start3p].northing, 0);
                if (end3p != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[end3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[end3p].northing, 0);
                GL.End();

                GL.PointSize(6);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(0.0f, 0.75f, 0.50f);  // first point
                if (start3p != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[start3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[start3p].northing, 0);

                GL.Color3(0.5f, 0.5f, 0.935f);  // second point
                if (end3p != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect3p].fenceLine[end3p].easting, mf.bnd.bndList[bndSelect3p].fenceLine[end3p].northing, 0);
            }
            else
            {
                GL.Color3(0, 0, 0);
                if (start3p != 99999) GL.Vertex3(ptA.easting, ptA.northing, 0);
                if (end3p != 99999) GL.Vertex3(ptB.easting, ptB.northing, 0);
                GL.End();

                GL.PointSize(10);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(.950f, 0.75f, 0.50f);
                if (start3p != 99999) GL.Vertex3(ptA.easting, ptA.northing, 0);

                GL.Color3(0.5f, 0.5f, 0.935f);
                if (end3p != 99999) GL.Vertex3(ptB.easting, ptB.northing, 0);
            }
            if (isC3p)  // 3rd point
            {
                GL.Color3(0.95f, 0.95f, 0.35f);
                GL.Vertex3(pint.easting, pint.northing, 0);
            }

            GL.End();
        }

        private void MakeABLineCurveBorder()
        {
            int starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int isStart = 0;
            int ik;
            ik = indx;

            tempListpart5.Clear();
            backupListABCurve.Clear();

            mf.bndl.tracksArrbndl.Add(new CBoundaryPath());
            mf.bndl.idxbndl = mf.bndl.tracksArrbndl.Count - 1;
            mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();


            //build the curve
            if (gTemp[ik].mode == (int)TrackMode.Curve)  // Curve of choise from list
            {
                vec3 PointsABCurve = new vec3(0.0, 0.0, 0.0);

                if ((double)nudSetDistancebndl.Value > ((mf.tool.width - mf.tool.overlap) + 3) * mf.m2FtOrM)
                {
                    nudSetDistancebndl.Value = (decimal)(((mf.tool.width - mf.tool.overlap) + 3) * mf.m2FtOrM);
                    nudSetDistancebndl.Text = (((mf.tool.width - mf.tool.overlap) + 3) * mf.m2FtOrM).ToString();
                    mf.TimedMessageBox(3000, "max more than 3 " + mf.unitsFtM + " ", " as toolwith    ");
                }

                for (int ic = 0; ic < gTemp[ik].curvePts.Count - 1; ic++)
                {
                    PointsABCurve.easting = gTemp[ik].curvePts[ic].easting - Math.Sin(glm.PIBy2 + gTemp[ik].curvePts[ic].heading) * (double)nudSetDistancebndl.Value * -mf.m2FtOrM;
                    PointsABCurve.northing = gTemp[ik].curvePts[ic].northing - Math.Cos(glm.PIBy2 + gTemp[ik].curvePts[ic].heading) * (double)nudSetDistancebndl.Value * -mf.m2FtOrM;
                    PointsABCurve.heading = gTemp[ik].curvePts[ic].heading;
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(PointsABCurve);
                }

                tempListpart4.Clear();
                tempListpart6.Clear();
                foreach (var item in mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl)
                {
                    tempListpart4.Add(item);  // save the moved ABline
                }

                if (mf.bndl.idxbndl > -1)
                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count < 2) return;

                if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > -1)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Clear();
                    foreach (var item in mf.bnd.bndList[bndSelect].fenceLine)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                    }
                }
                //save a backup fenceline
                backupList?.Clear();
                foreach (var item in mf.bnd.bndList[bndSelect].fenceLine_new)
                {
                    backupList.Add(item);
                }

                for (int i = 0; i < mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 2; i++)
                {
                    for (int k = 0; k < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 2; k++)
                    {
                        int res = GetLineIntersectionbndl(
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].northing,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].northing,

                        mf.bnd.bndList[bndSelect].fenceLine_new[k].easting,
                        mf.bnd.bndList[bndSelect].fenceLine_new[k].northing,
                        mf.bnd.bndList[bndSelect].fenceLine_new[k + 1].easting,
                        mf.bnd.bndList[bndSelect].fenceLine_new[k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0)
                            {
                                starttrack = i;
                                startbndl = k + 1;
                                // insert crossing startpoint in cutline
                                vec3 pointcrosstrackstart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].heading);
                                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(starttrack + 1, pointcrosstrackstart);

                                // insert crossing startpoint in fenceline_new
                                vec3 pointcrossfencestart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bnd.bndList[0].fenceLine_new[k].heading);
                                mf.bnd.bndList[bndSelect].fenceLine_new.Insert(startbndl, pointcrossfencestart);

                                CrossingpointEast1 = CrossingpointEast;
                                CrossingpointNorth1 = CrossingpointNorth;
                                CrossingpointEast = 0;
                                CrossingpointNorth = 0;
                            }
                            else
                            {
                                endtrack = i + 1;
                                endbndl = k;  // ############### + 1
                                if (CrossingpointEast != 0 && CrossingpointNorth != 0)
                                {
                                    CrossingpointEast2 = CrossingpointEast;
                                    CrossingpointNorth2 = CrossingpointNorth;

                                }
                            }

                            isStart++;

                        }
                    }
                }


                // insert crossing endpoint in cutline
                vec3 pointcrosstrackend = new vec3(CrossingpointEast2, CrossingpointNorth2, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].heading);
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(endtrack, pointcrosstrackend);
                mf.bnd.bndList[bndSelect].fenceLine_new.Insert(endbndl, pointcrosstrackend);

                for (int i = starttrack + 2; i < endtrack + 2; i++)
                {
                    tempListpart6.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
                }

                //for (int l = 0; l < mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count; l++)
                //    Console.WriteLine("mf.bndl.tracksArrbndl 0 : " + l + " " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[l].easting + " : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[l].northing);

                //                   Console.WriteLine("fenceLine_new 0 :  " + mf.bnd.bndList[bndSelect].fenceLine_new[startbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[startbndl].northing);
                //                   Console.WriteLine("fenceLine_new 1 :  " + mf.bnd.bndList[bndSelect].fenceLine_new[endbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[endbndl].northing);


                if (isStart < 2)
                {
                    mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                    mf.bndl.tracksArrbndl.Clear();
                    return;
                }
            }   // end of ABcurve  mode

            else if (gTemp[ik].mode == (int)TrackMode.AB)  // Line of choise from list
            {
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();
                int NewABLength = 200;

                vec3 PointsABLine = new vec3(0.0, 0.0, 0.0);
                vec3 PointsABLine1 = new vec3(0.0, 0.0, 0.0);
                PointsABLine.easting = gTemp[ik].ptA.easting - (Math.Sin(gTemp[ik].heading) * NewABLength) + Math.Cos(gTemp[ik].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM; // / 4 * 3;
                PointsABLine.northing = gTemp[ik].ptA.northing - (Math.Cos(gTemp[ik].heading) * NewABLength) - Math.Sin(gTemp[ik].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM; // / 4 * 3;
                PointsABLine.heading = gTemp[ik].heading;
                mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(PointsABLine);
                for (int iPoi = 1; iPoi < 150; iPoi++)
                {
                    PointsABLine1.easting = PointsABLine.easting + (Math.Sin(gTemp[ik].heading) * 25 * iPoi); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    PointsABLine1.northing = PointsABLine.northing + (Math.Cos(gTemp[ik].heading) * 25 * iPoi); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    PointsABLine1.heading = gTemp[ik].heading;
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(PointsABLine1);
                }

                tempListpart6.Clear();

                if (mf.bndl.idxbndl > -1)
                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count < 2) return;

                if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > -1)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Clear();
                    foreach (var item in mf.bnd.bndList[bndSelect].fenceLine)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                    }
                }

                //save a backup fenceline
                backupList?.Clear();
                foreach (var item in mf.bnd.bndList[bndSelect].fenceLine_new)
                {
                    backupList.Add(item);
                }

                vec3 pointcrosstrackstart = new vec3(0.0, 0.0, 0.0);
                tempListpart5.Clear();

                for (int i = 0; i < mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 2; i++)
                {
                    for (int k = 0; k < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 2; k++)
                    {
                        int res = GetLineIntersectionbndl(
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].northing,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].easting,
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].northing,

                        mf.bnd.bndList[0].fenceLine_new[k].easting,
                        mf.bnd.bndList[0].fenceLine_new[k].northing,
                        mf.bnd.bndList[0].fenceLine_new[k + 1].easting,
                        mf.bnd.bndList[0].fenceLine_new[k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0)
                            {
                                starttrack = i + 1;
                                startbndl = k + 1;
                            }
                            else
                            {
                                endtrack = i;
                                endbndl = k + 1;
                            }
                            isStart++;
                        }
                    }
                }

                if (isStart < 2)
                {
                    mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                    return;
                }


            }           // end of ABline  mode

            // start for all lines
            // start build off new fence line parts
            tempList1.Clear();
            tempList2.Clear();
            tempListpart5.Clear();

            for (int i = starttrack - 1; i < endtrack + 2; i++)
            {
                tempListpart5.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }


            // original direction of trackline 
            tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[startbndl]);
            for (int i = starttrack; i < endtrack; i++)
            {
                tempList1.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }
            // opposite direction of trackline 
            tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[endbndl]);
            tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[endbndl]);
            for (int i = endtrack; i > starttrack; i--)
            {
                tempList2.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
            }
            tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[startbndl]);

            if (startbndl > endbndl)
            {
                (startbndl, endbndl) = (endbndl, startbndl);
                (tempList1, tempList2) = (tempList2, tempList1);
            }

            // fence with Start&End
            // first fenceline_new segment is cutline part
            // tempList1
            // second segment
            for (int i = endbndl; i < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 1; i++)
            {
                tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }
            // dirth segment
            for (int i = 0; i < startbndl; i++)
            {
                tempList1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }
            // fence without Start&End
            // first fenceline_new segment is cutline part
            // tempList2
            // second segment
            for (int i = startbndl; i < endbndl + 1; i++)
            {
                tempList2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }

            mf.CalculateMinMax();
            CalculateFenceArea();
        }

        private void MakeABLineCurveforBorder() // load all line & curves for AB boundary  
        {
            HelpList?.Clear();
            for (int ikx = 0; ikx < gTemp.Count; ikx++)
            {
                vec3 PointsABCurve = new vec3(0.0, 0.0, 0.0);
                vec3 PointsABCurve1 = new vec3(0.0, 0.0, 0.0);

                //load all the AB Lines
                if (gTemp[ikx].mode == (int)TrackMode.AB)
                {
                    HelpList.Add(new List<vec3>());
                    HelpListTrim.Add(new List<vec3>());
                    // first point of AB line
                    PointsABCurve.easting = gTemp[ikx].ptA.easting - Math.Sin(gTemp[ikx].heading) * 400;
                    PointsABCurve.northing = gTemp[ikx].ptA.northing - Math.Cos(gTemp[ikx].heading) * 400;
                    PointsABCurve.heading = gTemp[ikx].heading;
                    HelpList[ikx].Add(PointsABCurve);

                    double Pointdiff = 0;
                    while (Pointdiff < 2000)
                    {
                        Pointdiff += 20;
                        PointsABCurve.easting += Math.Sin(gTemp[ikx].heading) * 30;
                        PointsABCurve.northing += Math.Cos(gTemp[ikx].heading) * 30;
                        PointsABCurve.heading = gTemp[ikx].heading;
                        HelpList[ikx].Add(PointsABCurve);
                    }
                }

                //load all the curves
                if (gTemp[ikx].mode == (int)TrackMode.Curve)  // Curve of choise from list
                {
                    for (int ic = 0; ic < gTemp[ikx].curvePts.Count - 1; ic++)
                    {
                        PointsABCurve.easting = gTemp[ikx].curvePts[ic].easting;
                        PointsABCurve.northing = gTemp[ikx].curvePts[ic].northing;
                        PointsABCurve.heading = gTemp[ikx].curvePts[ic].heading;
                        HelpList[ikx].Add(PointsABCurve);
                    }
                }
            }

        }


        private void DrawAllLinesA() // draws AB Lines curve lines and moved lines for AB boundary 
        {
            GL.LineStipple(1, 0x0707);

            for (int ikx = 0; ikx < HelpList.Count; ikx++)
            {
                if ((HelpList.Count > 2) && (HelpList[ikx].Count > 1))
                {
                    // Console.WriteLine("HelpList[ikx]  3  " + ikx + "  " + HelpList[ikx].Count);

                    nudgeLineBoundary[ikx] = 10; // (double)nudSetDistancebndl.Value;

                    if (gTemp[ikx].mode == (int)TrackMode.AB)
                    {
                        //GL.Enable(EnableCap.LineStipple);
                        GL.LineWidth(2);


                        //GL.Color3(0.30f, 0.7f, 1.30f);      // blue
                        GL.Color3(2.0f, 0.20f, 0.80f);         // rosa
                        GL.Begin(PrimitiveType.Lines);
                        //GL.Vertex3(HelpList[ikx][0].easting - (Math.Sin(HelpList[ikx][0].heading) * mf.ABLine.abLength), HelpList[ikx][0].northing - (Math.Cos(HelpList[ikx][0].heading) * mf.ABLine.abLength), 0);
                        //GL.Vertex3(HelpList[ikx][HelpList[ikx].Count - 1].easting + (Math.Sin(HelpList[ikx][HelpList[ikx].Count - 1].heading) * mf.ABLine.abLength), HelpList[ikx][HelpList[ikx].Count - 1].northing + (Math.Cos(HelpList[ikx][HelpList[ikx].Count - 1].heading) * mf.ABLine.abLength), 0);
                        GL.Vertex3(HelpList[ikx][0].easting, HelpList[ikx][0].northing, 0);
                        GL.Vertex3(HelpList[ikx][HelpList[ikx].Count - 1].easting, HelpList[ikx][HelpList[ikx].Count - 1].northing, 0);
                        GL.End();

                        if (nudgeLineBoundary[ikx] != 0)
                        {
                            GL.LineWidth(2);
                            GL.Color3(1f, 0.2f, 1f);      // red
                            GL.Begin(PrimitiveType.Lines);
                            //GL.Vertex3(HelpList[ikx][0].easting + Math.Cos(HelpList[ikx][0].heading) * nudgeLineBoundary[ikx] * mf.m2FtOrM, HelpList[ikx][0].northing - Math.Sin(HelpList[ikx][0].heading) * nudgeLineBoundary[ikx] * mf.m2FtOrM, 0);
                            //GL.Vertex3(HelpList[ikx][HelpList[ikx].Count - 1].easting + Math.Cos(HelpList[ikx][HelpList[ikx].Count - 1].heading) * nudgeLineBoundary[ikx] * mf.m2FtOrM, HelpList[ikx][HelpList[ikx].Count - 1].northing - Math.Sin(HelpList[ikx][HelpList[ikx].Count - 1].heading) * nudgeLineBoundary[ikx] * mf.m2FtOrM, 0);
                            GL.End();
                        }
                        // draws the violett AB buildline
                        GL.LineWidth(5);               // cut line
                        GL.Color3(1.0f, 0.0f, 1.0f);   //violett  tempListpart5
                        GL.Begin(PrimitiveType.Lines);
                        int maxCount = tempListpart5.Count - 2;
                        if (tempListpart5.Count > 1)
                        {
                            //GL.Vertex3(tempListpart5[0].easting, tempListpart5[0].northing, 0);
                            //GL.Vertex3(tempListpart5[maxCount].easting, tempListpart5[maxCount].northing, 0);
                        }
                        //}

                        GL.End();

                        //GL.Disable(EnableCap.LineStipple);

                    }
                    else if (gTemp[ikx].mode == (int)TrackMode.Curve)
                    {
                        GL.LineWidth(2);                    // for Draw of AB Curve
                        GL.Color3(0.30f, 0.7f, 1.30f);      // green
                        GL.Disable(EnableCap.LineStipple);
                        GL.Begin(PrimitiveType.LineStrip);

                        if (HelpList[ikx].Count > 1)
                        {
                            //foreach (vec3 item in mf.bndl.tracksArrbndl[ikx].trackPtsbndl) 
                            foreach (vec3 item in HelpList[ikx])

                            {
                                //GL.Vertex3(item.easting, item.northing, 0);   // Draw AB Curve
                                GL.Vertex3(item.easting, item.northing, 0);   // Draw AB Curve
                            }
                        }
                        GL.End();

                        if (nudgeLineBoundary[ikx] != 0)
                        {
                            GL.LineWidth(2);

                            GL.Color3(2.30f, 0.2f, 0.30f);      // red
                            GL.Disable(EnableCap.LineStipple);
                            GL.Begin(PrimitiveType.LineStrip);

                            foreach (vec3 pts in HelpList[ikx])
                            {
                                GL.Vertex3(pts.easting + Math.Cos(pts.heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, pts.northing - Math.Sin(pts.heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, 0);   // Draw moved AB Curve
                            }
                        }
                        GL.Disable(EnableCap.LineStipple);
                        GL.End();

                        GL.LineWidth(2);
                        GL.Color3(1.0f, 0.0f, 1.0f);   //violett
                        GL.Begin(PrimitiveType.LineStrip);

                        // if (HelpListTrim.Count > 0)
                        {

                            //    foreach (vec3 item in HelpListTrim[0])
                            //foreach (vec3 item in mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl)
                            {
                                //      GL.Vertex3(item.easting, item.northing, 0);    // draws the violett buildcurve
                            }
                        }
                    }
                }
            }
            //oglSelfbndl.SwapBuffers();
            GL.Disable(EnableCap.LineStipple);
            GL.End();
            mf.bndl.idxbndl = 1;
        }

        private void DrawBuiltLinesA() // draws AB Lines curve lines and moved lines 
        {
            GL.LineStipple(1, 0x0707);

            int i;
            i = indx;
            //           {
            //AB Lines
            if (gTemp[i].mode == (int)TrackMode.AB)
            {
                //GL.Enable(EnableCap.LineStipple);
                GL.LineWidth(2);

                //GL.Color3(2.0f, 0.20f, 0.20f);      // red
                GL.Color3(0.30f, 0.7f, 1.30f);      // green

                GL.Begin(PrimitiveType.Lines);

                GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);


                if (nudSetDistancebndl.Value != 0)
                {
                    GL.LineWidth(2);

                    GL.Color3(2.30f, 0.2f, 0.30f);      // red

                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, 0);
                    GL.End();
                }
                // draws the violett AB buildline
                GL.LineWidth(5);               // cut line
                GL.Color3(1.0f, 0.0f, 1.0f);   //violett  tempListpart5
                GL.Begin(PrimitiveType.Lines);
                int maxCount = tempListpart5.Count - 2;
                if (tempListpart5.Count > 1)
                {
                    GL.Vertex3(tempListpart5[0].easting, tempListpart5[0].northing, 0);
                    GL.Vertex3(tempListpart5[maxCount].easting, tempListpart5[maxCount].northing, 0);
                }
                //}

                GL.End();

                GL.Disable(EnableCap.LineStipple);

            }
            else if (gTemp[i].mode == (int)TrackMode.Curve)
            {
                GL.LineWidth(2);                    // for Draw of AB Curve
                GL.Color3(0.30f, 0.7f, 1.30f);      // green
                GL.Disable(EnableCap.LineStipple);
                GL.Begin(PrimitiveType.LineStrip);

                if (gTemp[i].curvePts.Count > 1)
                {
                    foreach (vec3 pts in gTemp[i].curvePts)
                    {
                        GL.Vertex3(pts.easting, pts.northing, 0);   // Draw AB Curve
                    }
                }
                GL.End();

                if (nudSetDistancebndl.Value != 0)
                {
                    GL.LineWidth(2);

                    GL.Color3(2.30f, 0.2f, 0.30f);      // red
                    GL.Disable(EnableCap.LineStipple);
                    GL.Begin(PrimitiveType.LineStrip);

                    foreach (vec3 pts in gTemp[i].curvePts)
                    {
                        GL.Vertex3(pts.easting + Math.Cos(pts.heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, pts.northing - Math.Sin(pts.heading) * (double)nudSetDistancebndl.Value * mf.m2FtOrM, 0);   // Draw moved AB Curve
                    }
                }
                GL.Disable(EnableCap.LineStipple);
                GL.End();

                GL.LineWidth(2);
                GL.Color3(1.0f, 0.0f, 1.0f);   //violett
                GL.Begin(PrimitiveType.LineStrip);

                if (mf.bndl.tracksArrbndl.Count > 0)
                {

                    foreach (vec3 item in tempListpart6)
                    //foreach (vec3 item in mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl)
                    {
                        GL.Vertex3(item.easting, item.northing, 0);    // draws the violett buildcurve
                    }
                }
            }
            GL.Disable(EnableCap.LineStipple);
            GL.End();

        }

        private void DrawBuiltLines()  // draws buildlines and new fenceline 
        {
            if (isLinesVisible && mf.bndl.tracksArrbndl.Count > 0)
            {
                //GL.Enable(EnableCap.LineStipple);
                GL.LineStipple(1, 0x7070);
                GL.PointSize(3);             //draw buildlines between points

                for (int i = 0; i < mf.bndl.tracksArrbndl.Count; i++)
                {
                    if (mf.bndl.tracksArrbndl[i].mode == (int)TrackMode.AB)
                    {
                        GL.Color3(0.973f, 0.9f, 0.10f);
                    }
                    else
                    {
                        GL.Color3(0.3f, 0.99f, 0.20f);
                    }
                    GL.Begin(PrimitiveType.Points);
                    foreach (vec3 item in mf.bndl.tracksArrbndl[i].trackPtsbndl)
                    {
                        GL.Vertex3(item.easting, item.northing, 0);
                    }
                    GL.End();
                }

                //GL.Disable(EnableCap.LineStipple);

                if (mf.bndl.idxbndl > -1)
                {
                    GL.LineWidth(4);   // draw buildlines
                    GL.Color3(1.0f, 0.0f, 1.0f);

                    GL.Begin(PrimitiveType.LineStrip);
                    foreach (vec3 item in mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl)
                    //foreach (vec3 item in tempListpart4)
                    {
                        GL.Vertex3(item.easting, item.northing, 0);    // draws the violett buildline
                    }
                    GL.End();

                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count > 1)
                    {
                        int cnt = mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 1;
                        GL.PointSize(10);
                        GL.Color3(0, 0, 0);
                        GL.Begin(PrimitiveType.Points);
                        GL.Vertex3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0].easting, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0].northing, 0);
                        GL.Color3(0, 0, 0);
                        GL.Vertex3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[cnt].easting, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[cnt].northing, 0);
                        GL.End();

                        GL.PointSize(10);
                        GL.Color3(1.0f, 0.7f, 0.35f);
                        GL.Begin(PrimitiveType.Points);
                        GL.Vertex3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0].easting, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0].northing, 0);
                        GL.Color3(0.6f, 0.75f, 0.99f);
                        GL.Vertex3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[cnt].easting, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[cnt].northing, 0);
                        GL.End();
                    }
                }
            }

            GL.LineWidth(6);
            GL.Color3(0.93f, 0.899f, 0.50f);
            //GL.Color3(0.93f, 0.0f, 0.00f);
            GL.Begin(PrimitiveType.LineStrip); // LineStrip  

            if (mf.bnd.bndList.Count > 0)
            {
                for (int i = 0; i < mf.bnd.bndList[bndSelect].fenceLine_new.Count; i++)
                {
                    GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine_new[i].easting, mf.bnd.bndList[bndSelect].fenceLine_new[i].northing, 0);///new fenceLine
                    // Console.WriteLine("Point " + i + " : " + mf.bnd.bndList[0].fenceLine_new[i].easting + " : " + mf.bnd.bndList[0].fenceLine_new[i].northing + " : " + mf.bnd.bndList[0].fenceLine_new[i].heading);
                }
            }
            GL.End();
        }

        private void DrawABTouchLine()
        {
            GL.Color3(0.65, 0.650, 0.0);
            GL.PointSize(24);
            GL.Begin(PrimitiveType.Points);

            if (mf.bnd.bndList.Count != 0)
            {
                GL.Color3(0, 0, 0);
                if (start != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[start].easting, mf.bnd.bndList[bndSelect].fenceLine[start].northing, 0);
                if (end != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[end].easting, mf.bnd.bndList[bndSelect].fenceLine[end].northing, 0);
                GL.End();

                GL.PointSize(4);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(1.0f, 0.75f, 0.350f);
                if (start != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[start].easting, mf.bnd.bndList[bndSelect].fenceLine[start].northing, 0);

                GL.Color3(0.5f, 0.75f, 1.0f);
                if (end != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[end].easting, mf.bnd.bndList[bndSelect].fenceLine[end].northing, 0);
                GL.End();
            }
            else
            {
                GL.Color3(0, 0, 0);
                if (start != 99999) GL.Vertex3(ptA.easting, ptA.northing, 0);
                if (end != 99999) GL.Vertex3(ptB.easting, ptB.northing, 0);
                GL.End();

                GL.PointSize(6);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(.950f, 0.75f, 0.50f);
                if (start != 99999) GL.Vertex3(ptA.easting, ptA.northing, 0);

                GL.Color3(0.5f, 0.5f, 0.935f);
                if (end != 99999) GL.Vertex3(ptB.easting, ptB.northing, 0);
            }
            if (isC)
            {
                GL.Color3(0.95f, 0.95f, 0.35f);
                GL.Vertex3(pint.easting, pint.northing, 0);
            }

            GL.End();
        }

        private void btnBndLoopbndl_Click(object sender, EventArgs e)
        {
            if ((cboxInfoButtons.Checked))//  || (mf.bndl.idxbndl < 0))
            {
                mf.TimedMessageBox(3000, "built a new boundaryline", "                  ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                MakeABLineCurveRealBorder();
            }
            else
            {
                if (!btnSlice.Checked)
                    if (le_do_half == 0)
                        BNTbndLoopbndl();
                    else
                    {
                        btn_Area1.Visible = true;
                        btn_Area2.Visible = true;
                        btn_Area1.BackColor = Color.LightPink;
                        btn_Area2.BackColor = Color.LightPink;
                        label2.Visible = true;
                        label3.Visible = true;
                        if (!cboxABLineCurve.Checked)
                            btnSlice_Line_Click();
                        else
                            MakeABLineCurveBorder();
                    }
                else
                    btnSliceBoundaryAch();
                btnBndLoopbndl.BackColor = Color.Transparent;
            }
        }

        private void Koordinat_Insert(int lineNum, int position, double pointEast, double pointNorth)
        {
            int positionlow = 0;
            double distance1, distance2, distance3, distance4, distance5;
            if (position > 2) positionlow = position - 3;

            vec3 pointcrosstrackstart = new vec3(CrossingpointEast, CrossingpointNorth, 0);

            distance1 = Math.Abs(mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[positionlow].easting - pointEast);
            distance2 = Math.Abs(mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[positionlow + 1].easting - pointEast);
            distance3 = Math.Abs(mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[positionlow + 2].easting - pointEast);
            distance4 = Math.Abs(mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[positionlow + 3].easting - pointEast);
            distance5 = Math.Abs(mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[positionlow + 4].easting - pointEast);
            if (distance1 < distance2)
            {
                if ((distance1 < distance2) && (distance2 > distance3))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 1, pointcrosstrackstart);
                }
                else if ((distance2 < distance3) && (distance3 > distance4))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 2, pointcrosstrackstart);
                }
                else if ((distance3 < distance4) && (distance4 > distance5))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 3, pointcrosstrackstart);
                }
            }
            else
            {
                if ((distance1 > distance2) && (distance2 < distance3))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 1, pointcrosstrackstart);
                }
                else if ((distance2 > distance3) && (distance3 < distance4))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 2, pointcrosstrackstart);
                }
                else if ((distance3 > distance4) && (distance4 < distance5))
                {
                    mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Insert(positionlow + 3, pointcrosstrackstart);
                }
            }
        }

        public void BNTbndLoopbndl()
        {
            //sort the lines
            mf.bndl.tracksArrbndl.Sort((p, q) => p.a_point.CompareTo(q.a_point));
            //mf.FileSaveBoundary();

            mf.bndl.idxbndl = -1;

            int numOfLines = mf.bndl.tracksArrbndl.Count;
            int nextLine = 0;
            crossingsbndl.Clear();

            int isStart = 0;
            int starttrack1 = 0, endtrack1 = 0, begintrack1 = 0, begintrack2 = 0;

            for (int lineNum = 0; lineNum < mf.bndl.tracksArrbndl.Count; lineNum++)
            {
                nextLine = lineNum - 1;
                if (nextLine < 0) nextLine = mf.bndl.tracksArrbndl.Count - 1;

                if (nextLine == lineNum)
                {
                    mf.TimedMessageBox(2000, "Create Error", "Is there maybe only 1 line?");
                    return;
                }

                for (int i = 0; i < mf.bndl.tracksArrbndl[lineNum].trackPtsbndl.Count - 2; i++)
                {
                    for (int k = 0; k < mf.bndl.tracksArrbndl[nextLine].trackPtsbndl.Count - 2; k++)
                    {
                        int res = GetLineIntersectionbndl(
                        mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[i].easting,
                        mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[i].northing,
                        mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[i + 1].easting,
                        mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[i + 1].northing,

                        mf.bndl.tracksArrbndl[nextLine].trackPtsbndl[k].easting,
                        mf.bndl.tracksArrbndl[nextLine].trackPtsbndl[k].northing,
                        mf.bndl.tracksArrbndl[nextLine].trackPtsbndl[k + 1].easting,
                        mf.bndl.tracksArrbndl[nextLine].trackPtsbndl[k + 1].northing,
                        ref iE, ref iN);

                        if (res == 1)
                        {
                            if (isStart == 0)
                            {
                                i++;
                                starttrack1 = i;
                                begintrack1 = k;
                                // create crossing startpoint in lineNum
                                vec3 pointcrosstrackstart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bndl.tracksArrbndl[lineNum].trackPtsbndl[i].heading);
                                // create crossing startpoint in nextLine
                                vec3 pointcrossfencestart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bndl.tracksArrbndl[nextLine].trackPtsbndl[k].heading);
                                // insert crossingpoints at right position in Line
                                Koordinat_Insert(lineNum, starttrack1, CrossingpointEast, CrossingpointNorth);
                                Koordinat_Insert(nextLine, begintrack1, CrossingpointEast, CrossingpointNorth);

                                CrossingpointEast = 0;
                                CrossingpointNorth = 0;

                            }
                            crossingsbndl.Add(i);
                            isStart++;

                            if (isStart == 2)
                            {
                                endtrack1 = i;
                                begintrack2 = k;
                                goto again;
                            }
                            nextLine = lineNum + 1;

                            if (nextLine > mf.bndl.tracksArrbndl.Count - 1) nextLine = 0;
                        }
                    }

                }            // insert crossing endpoint in cutline

                again:
                isStart = 0;
            }

            if (crossingsbndl.Count != mf.bndl.tracksArrbndl.Count * 2)
            {
                Console.WriteLine(" crossingsbndl.Count : " + crossingsbndl.Count);
                Console.WriteLine(" mf.bndl.tracksArrbndl.Count  : " + mf.bndl.tracksArrbndl.Count);
                mf.TimedMessageBox(2000, "Crosings Error", "Make sure all ends cross only once");
                //mf.bnd.bndList[0].fenceLine_new?.Clear();
                return;
            }

            //build the new fenceline
            mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();

            for (int i = 0; i < mf.bndl.tracksArrbndl.Count; i++)
            {
                int low = crossingsbndl[i * 2];
                int high = crossingsbndl[i * 2 + 1];
                for (int k = low; k < high; k++)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(mf.bndl.tracksArrbndl[i].trackPtsbndl[k]);
                }
            }

            //Console.WriteLine("mf.bnd.bndList[0].fenceLine_new : " + mf.bnd.bndList[0].fenceLine_new);

            vec3[] hdArrbndl;

            if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > 0)
            {
                hdArrbndl = new vec3[mf.bnd.bndList[bndSelect].fenceLine_new.Count];
                mf.bnd.bndList[bndSelect].fenceLine_new.CopyTo(hdArrbndl);
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();
            }
            else
            {
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();
                return;
            }

            //middle points
            for (int i = 1; i < hdArrbndl.Length; i++)
            {
                hdArrbndl[i - 1].heading = Math.Atan2(hdArrbndl[i - 1].easting - hdArrbndl[i].easting, hdArrbndl[i - 1].northing - hdArrbndl[i].northing);
                if (hdArrbndl[i].heading < 0) hdArrbndl[i].heading += glm.twoPI;
            }

            double delta = 0;
            for (int i = 0; i < hdArrbndl.Length; i++)
            {
                if (i == 0)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(new vec3(hdArrbndl[i].easting, hdArrbndl[i].northing, hdArrbndl[i].heading));
                    continue;
                }
                delta += (hdArrbndl[i - 1].heading - hdArrbndl[i].heading);

                if (Math.Abs(delta) > 0.005)
                {
                    vec3 pt = new vec3(hdArrbndl[i].easting, hdArrbndl[i].northing, hdArrbndl[i].heading);

                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(pt);
                    delta = 0;
                }
            }
            if (mf.bnd.bndList.Count > 0)
            {
                GL.LineWidth(6);
                if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > 3)
                {
                    GL.Color3(0.0555f, 1.6232f, 0.20f);
                    mf.bnd.bndList[bndSelect].fenceLine_new.DrawPolygon();
                    //isStep = true;
                }
                GL.End();
            }

            mf.CalculateMinMax();
            CalculateFenceArea();

            //mf.FileSaveBoundary();
        }

        private void btnSliceBoundaryAch()
        {
            bool isLoop = false;
            int limit = end3p;
            if (end3p == 99999 || start3p == 99999) return;

            if (mf.bnd.bndList[bndSelect3p].fenceLine.Count > 0)
            {
                if ((Math.Abs(start3p - end3p)) > (mf.bnd.bndList[bndSelect3p].fenceLine.Count * 0.5))
                {
                    isLoop = true;
                    if (start3p < end3p)
                    {
                        (end3p, start3p) = (start3p, end3p);
                    }

                    limit = end3p;
                    end3p = mf.bnd.bndList[bndSelect3p].fenceLine.Count;
                }
                else //normal
                {
                    if (start3p > end3p)
                    {
                        (end3p, start3p) = (start3p, end3p);
                    }
                }

                // makes a copy of original fence
                vec3[] arr = new vec3[mf.bnd.bndList[bndSelect3p].fenceLine.Count];
                mf.bnd.bndList[bndSelect3p].fenceLine.CopyTo(arr);

                if (start3p++ == arr.Length) start3p--;
                //if (end-- == -1) end = 0;
                if (start3p == end3p) return;

                for (int i = start3p; i < end3p; i++)
                {
                    //calculate the point inside the boundary
                    arr[i].heading = 999;

                    if (isLoop && i == mf.bnd.bndList[bndSelect3p].fenceLine.Count - 1)
                    {
                        i = -1;
                        isLoop = false;
                        end3p = limit;
                    }
                }

                arr[start3p] = new vec3(pint);


                //mf.bnd.bndList[bndSelect].fenceLine.Clear();
                mf.bnd.bndList[bndSelect3p].fenceLine_new.Clear();

                for (int i = 0; i < arr.Length; i++)
                {
                    //calculate the point inside the boundary
                    if (arr[i].heading != 999)
                        mf.bnd.bndList[bndSelect3p].fenceLine_new.Add(new vec3(arr[i]));

                    if (isLoop && i == arr.Length - 1)
                    {
                        i = -1;
                        isLoop = false;
                        end3p = limit;
                    }
                }

                FixFenceLine_new(bndSelect3p);

                mf.CalculateMinMax();
                CalculateFenceArea();

                mf.fd.UpdateFieldBoundaryGUIAreas();
                //mf.FileSaveBoundary();
            }

            start3p = 99999; end3p = 99999;
            isA3p = true;
            isC3p = false;

            //zoom = 1;
            //sX = 0;
            //sY = 0;
        }

        public void FixFenceLine_new(int bndNum)
        {
            double spacing;
            //close if less then 20 ha, 40ha, more
            if (area < 200000) spacing = 1.1;
            else if (area < 400000) spacing = 2.2;
            else spacing = 3.3;

            if (bndNum > 0) spacing *= 0.5;

            int bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
            double distance;

            //make sure distance isn't too big between points on boundary
            for (int i = 0; i < bndCount; i++)
            {
                int j = i + 1;

                if (j == bndCount) j = 0;
                distance = glm.Distance(mf.bnd.bndList[bndSelect3p].fenceLine_new[i], mf.bnd.bndList[bndSelect3p].fenceLine_new[j]);
                if (distance > spacing * 1.5)
                {
                    vec3 pointB = new vec3((mf.bnd.bndList[bndSelect3p].fenceLine_new[i].easting + mf.bnd.bndList[bndSelect3p].fenceLine_new[j].easting) / 2.0,
                        (mf.bnd.bndList[bndSelect3p].fenceLine_new[i].northing + mf.bnd.bndList[bndSelect3p].fenceLine_new[j].northing) / 2.0, mf.bnd.bndList[bndSelect3p].fenceLine_new[i].heading);

                    mf.bnd.bndList[bndSelect3p].fenceLine_new.Insert(j, pointB);
                    bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
                    i--;
                }
            }

            //make sure distance isn't too big between points on boundary
            bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;

            for (int i = 0; i < bndCount; i++)
            {
                int j = i + 1;

                if (j == bndCount) j = 0;
                distance = glm.Distance(mf.bnd.bndList[bndSelect3p].fenceLine_new[i], mf.bnd.bndList[bndSelect3p].fenceLine_new[j]);
                if (distance > spacing * 1.6)
                {
                    vec3 pointB = new vec3((mf.bnd.bndList[bndSelect3p].fenceLine_new[i].easting + mf.bnd.bndList[bndSelect3p].fenceLine_new[j].easting) / 2.0,
                        (mf.bnd.bndList[bndSelect3p].fenceLine_new[i].northing + mf.bnd.bndList[bndSelect3p].fenceLine_new[j].northing) / 2.0, mf.bnd.bndList[bndSelect3p].fenceLine_new[i].heading);

                    mf.bnd.bndList[bndSelect3p].fenceLine_new.Insert(j, pointB);
                    bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
                    i--;
                }
            }

            //make sure distance isn't too small between points on headland
            spacing *= 0.9;
            bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
            for (int i = 0; i < bndCount - 1; i++)
            {
                distance = glm.Distance(mf.bnd.bndList[bndSelect3p].fenceLine_new[i], mf.bnd.bndList[bndSelect3p].fenceLine_new[i + 1]);
                if (distance < spacing)
                {
                    mf.bnd.bndList[bndSelect3p].fenceLine_new.RemoveAt(i + 1);
                    bndCount = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
                    i--;
                }
            }

            //make sure headings are correct for calculated points
            CalculateFenceLineHeadings3p();
            /*
                        double delta = 0;
                        fenceLineEar?.Clear();

                        for (int i = 0; i < mf.bnd.bndList[bndSelect3p].fenceLine_new.Count; i++)
                        {
                            if (i == 0)
                            {
                                fenceLineEar.Add(new vec2(mf.bnd.bndList[bndSelect3p].fenceLine_new[i].easting, mf.bnd.bndList[bndSelect3p].fenceLine_new[i].northing));
                                continue;
                            }
                            delta += (mf.bnd.bndList[bndSelect3p].fenceLine_new[i - 1].heading - mf.bnd.bndList[bndSelect3p].fenceLine_new[i].heading);
                            if (Math.Abs(delta) > 0.005)
                            {
                                fenceLineEar.Add(new vec2(mf.bnd.bndList[bndSelect3p].fenceLine_new[i].easting, mf.bnd.bndList[bndSelect3p].fenceLine_new[i].northing));
                                delta = 0;
                            }
                        }*/
        }

        public void CalculateFenceLineHeadings3p()  // new fenceline with 3 point cut
        {
            //to calc heading based on next and previous points to give an average heading.
            int cnt = mf.bnd.bndList[bndSelect3p].fenceLine_new.Count;
            vec3[] arr = new vec3[cnt];
            cnt--;
            mf.bnd.bndList[bndSelect3p].fenceLine_new.CopyTo(arr);
            mf.bnd.bndList[bndSelect3p].fenceLine_new.Clear();

            //first point needs last, first, second points
            vec3 pt3 = arr[0];
            pt3.heading = Math.Atan2(arr[1].easting - arr[cnt].easting, arr[1].northing - arr[cnt].northing);
            if (pt3.heading < 0) pt3.heading += glm.twoPI;
            mf.bnd.bndList[bndSelect3p].fenceLine_new.Add(pt3);

            //middle points
            for (int i = 1; i < cnt; i++)
            {
                pt3 = arr[i];
                pt3.heading = Math.Atan2(arr[i + 1].easting - arr[i - 1].easting, arr[i + 1].northing - arr[i - 1].northing);
                if (pt3.heading < 0) pt3.heading += glm.twoPI;
                mf.bnd.bndList[bndSelect3p].fenceLine_new.Add(pt3);
            }

            //last and first point
            pt3 = arr[cnt];
            pt3.heading = Math.Atan2(arr[0].easting - arr[cnt - 1].easting, arr[0].northing - arr[cnt - 1].northing);
            if (pt3.heading < 0) pt3.heading += glm.twoPI;
            mf.bnd.bndList[bndSelect3p].fenceLine_new.Add(pt3);
        }

        private void btnSlice_Line_Click()  // cut boundary with one touch line 
        {
            int starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int isStart = 0;

            if (mf.bndl.idxbndl > -1)
                if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count < 2) return;

            if (mf.bnd.bndList[bndSelect].fenceLine_new.Count == 0)
                foreach (var item in mf.bnd.bndList[bndSelect].fenceLine)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                }


            //save a backup
            backupList?.Clear();
            foreach (var item in mf.bnd.bndList[bndSelect].fenceLine_new)
            {
                backupList.Add(item);
            }

            for (int i = 0; i < mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 2; i++)
            {
                for (int k = 0; k < mf.bnd.bndList[bndSelect].fenceLine_new.Count - 2; k++)
                {
                    int res = GetLineIntersectionbndl(
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].easting,
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].northing,
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].easting,
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i + 1].northing,

                    mf.bnd.bndList[bndSelect].fenceLine_new[k].easting,
                    mf.bnd.bndList[bndSelect].fenceLine_new[k].northing,
                    mf.bnd.bndList[bndSelect].fenceLine_new[k + 1].easting,
                    mf.bnd.bndList[bndSelect].fenceLine_new[k + 1].northing,
                    ref iE, ref iN);
                    if (res == 1)
                    {
                        if (isStart == 0)
                        {
                            starttrack = i;
                            startbndl = k;
                            // insert crossing startpoint in cutline
                            vec3 pointcrosstrackstart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i].heading);
                            mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(starttrack + 1, pointcrosstrackstart);
                            // insert crossing startpoint in fenceline_new
                            vec3 pointcrossfencestart = new vec3(CrossingpointEast, CrossingpointNorth, mf.bnd.bndList[bndSelect].fenceLine_new[k].heading);
                            mf.bnd.bndList[bndSelect].fenceLine_new.Insert(startbndl, pointcrossfencestart);
                            CrossingpointEast = 0;
                            CrossingpointNorth = 0;
                        }
                        else
                        {
                            endtrack = i + 1;
                            endbndl = k + 1;  // ############### + 1
                            if (CrossingpointEast != 0 && CrossingpointNorth != 0)
                            {
                                CrossingpointEast2 = CrossingpointEast;
                                CrossingpointNorth2 = CrossingpointNorth;
                            }
                        }

                        isStart++;
                    }
                }
            }


            // insert crossing endpoint in cutline
            vec3 pointcrosstrackend = new vec3(CrossingpointEast2, CrossingpointNorth2, mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].heading);
            mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(endtrack, pointcrosstrackend);
            mf.bnd.bndList[bndSelect].fenceLine_new.Insert(endbndl, pointcrosstrackend);

            if (isStart < 2)
            {
                mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                return;
            }

            // start build off new fence line
            tempList1.Clear();
            tempList2.Clear();
            tempListpart1.Clear();
            tempListpart2.Clear();
            tempListpart3.Clear();

            //first bnd segment  starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int crossingpoint1, crossingpoint2, crossingpoint3, crossingpoint4;
            // from fenceline_new 0 to crossingpoint
            if (endbndl < startbndl)
            {
                crossingpoint1 = endtrack;
                crossingpoint2 = starttrack;
                crossingpoint3 = endbndl;
                crossingpoint4 = startbndl;
                //first fenceline_new segment is cutline part
                for (int i = starttrack + 1; i < endtrack; i++)
                {
                    tempList1.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
                }
                for (int i = endtrack; i > starttrack; i--)
                {
                    tempList2.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
                }
            }
            else
            {
                crossingpoint1 = starttrack;
                crossingpoint2 = endtrack;
                crossingpoint3 = startbndl;
                crossingpoint4 = endbndl;
                //first fenceline_new segment is cutline part
                for (int i = starttrack + 1; i < endtrack; i++)
                {
                    tempList2.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
                }
                for (int i = endtrack; i > starttrack; i--)
                {
                    tempList1.Add(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[i]);
                }
            }
            // only cutline
            mf.bndl.tracksArrbndl[0].trackPtsbndl?.Clear();
            foreach (var item in tempList1)
            {
                mf.bndl.tracksArrbndl[0].trackPtsbndl.Add(item);
            }

            for (int i = 0; i <= crossingpoint3; i++)
            {
                tempListpart1.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }

            for (int i = crossingpoint3 + 1; i <= crossingpoint4 + 1; i++)
            {
                tempListpart2.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }

            for (int i = crossingpoint4 + 1; i < mf.bnd.bndList[bndSelect].fenceLine_new.Count; i++)
            {
                tempListpart3.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
            }

            // assamble the two lists to lines 1 and 2
            // list 1
            foreach (var item in tempListpart2)
            {
                tempList1.Add(item);
            }
            // list 2
            foreach (var item in tempList2)
            {
                tempListpart1.Add(item);
            }
            foreach (var item in tempListpart3)
            {
                tempListpart1.Add(item);
            }
            tempList2.Clear();
            foreach (var item in tempListpart1)
            {
                tempList2.Add(item);
            }
            CalculateFenceArea();
            //mf.bndl.tracksArrbndl[0].trackPtsbndl?.Clear();
            //btn_Area1.Visible = true;
            //btn_Area2.Visible = true;
        }

        public int GetLineIntersectionbndl(double p0x, double p0y, double p1x, double p1y,
             double p2x, double p2y, double p3x, double p3y, ref double iEast, ref double iNorth)
        {
            CrossingpointEast = 0;
            CrossingpointNorth = 0;

            double s1x, s1y, s2x, s2y;
            s1x = p1x - p0x;
            s1y = p1y - p0y;

            s2x = p3x - p2x;
            s2y = p3y - p2y;

            double s, t;
            s = (-s1y * (p0x - p2x) + s1x * (p0y - p2y)) / (-s2x * s1y + s1x * s2y);

            if (s >= 0 && s <= 1)
            {
                //check oher side
                t = (s2x * (p0y - p2y) - s2y * (p0x - p2x)) / (-s2x * s1y + s1x * s2y);
                if (t >= 0 && t <= 1)
                {
                    // Collision detected
                    iEast = p0x + (t * s1x);
                    iNorth = p0y + (t * s1y);

                    CrossingpointEast = iEast;
                    CrossingpointNorth = iNorth;
                    return 1;
                }

            }

            return 0; // No collision
        }

        private void btnSlice_line_CheckedChanged(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(3000, "split area with a", "free line");
                return;
            }
            else
            {
                if (btnSlice_line.Checked)
                {
                    le_do_half = 1;
                    if (!rbtnCurvebndl.Checked)
                        rbtnLinebndl.Checked = true;
                    /*              if (mf.bndl.tracksArrbndl[0].trackPtsbndl.Count > 0)
                                  {
                                      mf.bndl.tracksArrbndl[0].trackPtsbndl?.Clear();
                                      oglSelfbndl.Refresh();
                                  }
                       */
                    btnSlice.Checked = false;
                    btnSlice_line.Checked = true;
                    btn_Area1.Visible = true;
                    btn_Area2.Visible = true;
                    label2.Visible = true;
                    label3.Visible = true;

                    cboxABBoundary.Checked = false;
                    cboxABBoundary.BackColor = Color.Transparent;


                    return;
                }
                else
                {
                    le_do_half = 0;
                    btnSlice_line.Checked = false;
                    btn_Area1.Visible = false;
                    btn_Area2.Visible = false;
                    label2.Visible = false;
                    label3.Visible = false;
                    return;
                }
            }
        }

        private void timer1_Tick_1(object sender, EventArgs e)
        {
            oglSelfbndl.Refresh();
        }

        private void btnExitbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(3000, "Exit and", "save all");
            }
            else
            {
                // delete all helplines
                for (int i = 0; i < mf.bndl.tracksArrbndl.Count; i++)
                {
                    mf.bndl.tracksArrbndl[i].trackPtsbndl?.Clear();
                }

                if (mf.bnd.bndList[bndSelect].fenceLine_new.Count > 2)
                {
                    mf.bnd.bndList[bndSelect].fenceLine?.Clear();

                    for (int i = 0; i < mf.bnd.bndList[bndSelect].fenceLine_new.Count; i++)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine.Add(mf.bnd.bndList[bndSelect].fenceLine_new[i]);
                    }
                    mf.FileSaveBoundary();

                    // close the open field
                    mf.FileSaveEverythingBeforeClosingField();

                    //open the field again with new fenceline
                    mf.FileOpenField("Resume");

                    mf.fd.UpdateFieldBoundaryGUIAreas();  //#############################
                }
                Close();
            }
        }

        private void nudSetDistancebndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "how far away ", "the line is moved from line");
                return;
            }
            else
            {
                mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
                //btnExitbndl.Focus();
                if (cboxABLineCurve.Checked)
                {
                    mf.bndl.tracksArrbndl.Clear();
                    MakeABLineCurveBorder();

                    cboxABLineCurve.BackColor = Color.Transparent;
                }
            }
        }

        private void oglSelfbndl_Load(object sender, EventArgs e)
        {
            oglSelfbndl.MakeCurrent();
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.ClearColor(0.22f, 0.22f, 0.22f, 1.0f);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
        }

        private void btnALengthbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "Add line orange side", "              ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                int ptCnt = HelpListTrim[bndSelectTrim].Count - 1;

                if (TouchABPoints[bndSelectTrim].Count > 1)
                {
                    for (int i = 1; i < 20; i += 5)
                    {
                        //vec3 pt = new vec3(HelpListTrim[bndSelectTrim][ptCnt]);
                        vec3 pt = new vec3(HelpListTrim[bndSelectTrim][0]);
                        pt.easting -= (Math.Sin(pt.heading) * i);
                        pt.northing -= (Math.Cos(pt.heading) * i);
                        HelpListTrim[bndSelectTrim].Insert(0, pt);
                        TouchABPoints[bndSelectTrim].RemoveAt(0);
                        TouchABPoints[bndSelectTrim].Insert(0, pt);
                    }
                }
            }
            else
            {
                if (mf.bndl.idxbndl > -1)
                {
                    //and the beginning
                    vec3 start = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[0]);

                    for (int i = 1; i < 10; i++)
                    {
                        vec3 pt = new vec3(start);
                        pt.easting -= (Math.Sin(pt.heading) * i);
                        pt.northing -= (Math.Cos(pt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Insert(0, pt);
                    }
                }
            }
        }

        private void btnAShrinkbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "short line orange side", "              ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                if (HelpListTrim[bndSelectTrim].Count > 8)
                {
                    HelpListTrim[bndSelectTrim].RemoveRange(0, 5);
                    TouchABPoints[bndSelectTrim].RemoveAt(0);
                    TouchABPoints[bndSelectTrim].Insert(0, HelpListTrim[bndSelectTrim][0]);
                }
            }
            else
            {
                if (mf.bndl.idxbndl > -1)
                {
                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count > 8)
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.RemoveRange(0, 5);
                }
            }
        }

        private void btnBLengthbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "Add line blue side", "              ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                int ptCnt = HelpListTrim[bndSelectTrim].Count - 1;
                if (TouchABPoints[bndSelectTrim].Count > 1)
                {
                    for (int i = 1; i < 20; i += 5)
                    {

                        vec3 pt = new vec3(HelpListTrim[bndSelectTrim][ptCnt]);
                        pt.easting += (Math.Sin(pt.heading) * i);
                        pt.northing += (Math.Cos(pt.heading) * i);
                        HelpListTrim[bndSelectTrim].Add(pt);
                        TouchABPoints[bndSelectTrim].RemoveAt(1);
                        TouchABPoints[bndSelectTrim].Add(pt);
                    }
                }
            }
            else
            {
                if (mf.bndl.idxbndl > -1)
                {
                    int ptCnt = mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 1;

                    for (int i = 1; i < 10; i++)
                    {
                        vec3 pt = new vec3(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[ptCnt]);
                        pt.easting += (Math.Sin(pt.heading) * i);
                        pt.northing += (Math.Cos(pt.heading) * i);
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Add(pt);
                    }
                }
            }
        }

        private void btnBShrinkbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "short line blue side", "              ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                if (HelpListTrim[bndSelectTrim].Count > 8)
                {
                    HelpListTrim[bndSelectTrim].RemoveRange(HelpListTrim[bndSelectTrim].Count - 5, 5);
                    TouchABPoints[bndSelectTrim].RemoveAt(TouchABPoints[bndSelectTrim].Count - 1);
                    TouchABPoints[bndSelectTrim].Add(HelpListTrim[bndSelectTrim][HelpListTrim[bndSelectTrim].Count - 1]);
                }
            }
            else
            {
                if (mf.bndl.idxbndl > -1)
                {
                    if (mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count > 8)
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.RemoveRange(mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Count - 5, 5);
                }
            }
        }

        private void btnCancelTouchbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "delete touchpoints ", "             ");
                return;
            }
            else
            {
                //update the arrays
                start = 99999; end = 99999;
                start3p = 99999; end3p = 99999;
                isA = true;
                mf.curve.desList?.Clear();
                zoom = 1;
                sX = 0;
                sY = 0;
                btnExitbndl.Focus();
            }
        }

        private void btnDeleteYouturnbndl_Click(object sender, EventArgs e)
        {
            btnDeleteYouturnbndl_Click1();
        }

        private void btnDeleteYouturnbndl_Click1()
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "delete new boundary ", "                ");
                return;
            }
            else
            {
                start = 99999; end = 99999;
                start3p = 99999; end3p = 99999;
                isA = true;
                isA3p = true;
                if ((btnSlice_line.Checked && backupList.Count > 2) && (mf.bndl.idxbndl > -1))
                {
                    mf.bnd.bndList[bndSelect].fenceLine?.Clear();
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl?.Clear();

                    foreach (var item in backupList)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine.Add(item);
                    }
                    oglSelfbndl.Refresh();
                    CalculateFenceArea();

                }
                btnBndLoopbndl.BackColor = Color.Transparent;
                mf.bndl.desListbndl?.Clear();
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();

                tempList1.Clear();
                tempList2.Clear();
                btn_Area1.Visible = false;
                btn_Area2.Visible = false;
                btn_Area1.BackColor = Color.LightPink;
                btn_Area2.BackColor = Color.LightPink;
                label2.Visible = false;
                label3.Visible = false;
                lbldistance1.Visible = false;
                lbldistance2.Visible = false;

                if (mf.bndl.idxbndl < 1) btnSlice_line.Checked = true;
                else btnSlice_line.Checked = false;


            }
        }

        private void btnCycleBackwardbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "skip back a builtline ", "                ");
                return;
            }
            else
            {
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();

                if (mf.bndl.tracksArrbndl.Count > 0)
                {
                    mf.bndl.idxbndl--;
                    if (mf.bndl.idxbndl < 0) mf.bndl.idxbndl = (mf.bndl.tracksArrbndl.Count - 1);
                }
                else
                {
                    mf.bndl.idxbndl = -1;
                }
            }
        }

        private void btnCycleForwardbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "skip forward a builtline ", "                ");
                return;
            }
            else
            {
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();

                if (mf.bndl.tracksArrbndl.Count > 0)
                {
                    mf.bndl.idxbndl++;
                    if (mf.bndl.idxbndl > (mf.bndl.tracksArrbndl.Count - 1)) mf.bndl.idxbndl = 0;
                }
                else
                {
                    mf.bndl.idxbndl = -1;
                }
            }
        }

        private void cboxToolWidthsbndl_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "how many Toolwith", "to move line   ");
                return;
            }
            else
            {
                nudSetDistancebndl.Value = (decimal)((Math.Round((mf.tool.width - mf.tool.overlap) * cboxToolWidthsbndl.SelectedIndex, 1)) * mf.m2FtOrM);
                cboxABLineCurve.BackColor = Color.Transparent;
            }
        }

        private void btnHeadlandOffbndl_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "Leave Window", "without save");
                return;
            }
            else
            {
                btnDeleteCurvebndl1_Click();
                mf.vehicle.isHydLiftOn = false;
                Close();
            }
        }

        private void btnDeleteCurvebndl_Click(object sender, EventArgs e)
        {
            btnDeleteCurvebndl1_Click();
            if (mf.bndl.idxbndl == 0) btnSlice_line.Checked = true;
            else btnSlice_line.Checked = false;
        }

        private void btnDeleteCurvebndl1_Click()
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "delete violett builtline ", "                  ");
                return;
            }
            else  
            {
                if ((btnSlice_line.Checked && backupList.Count > 2) && (mf.bndl.idxbndl > 0))
                {
                    mf.bnd.bndList[bndSelect].fenceLine?.Clear();
                    mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl?.Clear();

                    foreach (var item in backupList)
                    {
                        mf.bnd.bndList[bndSelect].fenceLine.Add(item);
                    }
                    oglSelfbndl.Refresh();
                    CalculateFenceArea();

                }
                btnBndLoopbndl.BackColor = Color.Transparent;
                mf.bndl.desListbndl?.Clear();
                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();

                if (mf.bndl.tracksArrbndl.Count > 0 && mf.bndl.idxbndl > -1)
                {
                    mf.bndl.tracksArrbndl.RemoveAt(mf.bndl.idxbndl);
                    mf.bndl.idxbndl--;
                }

                if (mf.bndl.tracksArrbndl.Count > 0)
                {
                    if (mf.bndl.idxbndl == -1)
                    {
                        mf.bndl.idxbndl++;
                    }
                }
                else mf.bndl.idxbndl = -1;

                nudSetDistancebndl.Value = 0;
                cboxToolWidthsbndl.Text = "0";
                btn_Area1.Text = "0";
                btn_Area1.Visible = false;
                area1 = 0;
                btn_Area2.Text = "0";
                btn_Area2.Visible = false;
                area2 = 0;
                //startAB = 99999; endAB = 99999;
                TouchABPoints?.Clear();
                if (TouchABPoints.Count == 0)
                    for (int ikx = 0; ikx < gTemp.Count; ikx++)
                    {
                        HelpList.Add(new List<vec3>());
                        HelpListTrim.Add(new List<vec3>());
                        TouchABPoints.Add(new List<vec3>());
                    }

                HelpList?.Clear();
                HelpListTrim?.Clear();

            }
        }

        private void rbtnCurvebndl_CheckedChanged(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose builtline ", "tip start and end");
                return;
            }
            else
            {
                btnMoveUp.Visible = false;
                btnMoveDn.Visible = false;
                btnMoveLeft.Visible = false;
                btnMoveRight.Visible = false;

                lbldistance1.Visible = false;
                lbldistance2.Visible = false;
            }
        }

        private void rbtnLinebndl_CheckedChanged(object sender, EventArgs e)
        {
            btnMoveUp.Visible = false;
            btnMoveDn.Visible = false;
            btnMoveLeft.Visible = false;
            btnMoveRight.Visible = false;

            lbldistance1.Visible = false;
            lbldistance2.Visible = false;
        }

        private void btnSlice_CheckedChanged(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose builtline ", "tip for 3 points  ");
                return;
            }
            else
            {
                if (btnSlice.Checked)
                {
                    le_do_half = 0;
                    btnSlice_line.Checked = false;
                    cboxABLineCurve.Checked = false;
                    btnBndLoopbndl.Visible = true;
                }
            }
        }

        private void btnMovedistance()
        {
            dist1 = glm.Distance(mf.bnd.bndList[bndSelect].fenceLine[start3p], pint);
            lbldistance1.Text = Convert.ToString(dist1);
            dist2 = glm.Distance(mf.bnd.bndList[bndSelect].fenceLine[end3p], pint);
            lbldistance2.Text = Convert.ToString(dist2);

        }

        private void btnMoveUp_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "move 3.built ", "point up ");
                return;
            }
            else
            {
                pint.northing += 0.5;
                btnMovedistance();
            }
        }

        private void btnMoveDn_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "move 3.built ", "point down ");
                return;
            }
            else
            {
                pint.northing -= 0.4;
                btnMovedistance();
            }
        }

        private void btnMoveLeft_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "move 3.built ", "point left ");
                return;
            }
            else
            {
                pint.easting -= 0.4;
                btnMovedistance();
            }
        }

        private void btnMoveRight_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "move 3.built ", "point right ");
                return;
            }
            else
            {
                pint.easting += 0.5;
                btnMovedistance();
            }
        }

        private void btn_Area1_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose area to ", "be next new field ");
                return;
            }
            else
            {
                if (btn_Area1.BackColor == Color.GreenYellow) btn_Area1.BackColor = Color.LightPink;
                else btn_Area1.BackColor = Color.GreenYellow;
                btn_Area2.BackColor = Color.LightPink;

                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();
                foreach (var item in tempList1)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                }
                btn_Area2.Visible = false;
                area2 = 0;
                label3.Visible = false;
                cboxABLineCurve.Checked = false;
                btnCycleBackwardFirstAB.Visible = false;
                btnCycleForwardFirstAB.Visible = false;

                /*     
                if (cboxABLineCurve.Checked)
                DrawBuiltLinesA();
                else
                DrawBuiltLines();
                */
            }
        }

        private void btn_Area2_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose area to ", "be next new field ");
                return;
            }
            else
            {
                if (btn_Area2.BackColor == Color.GreenYellow) btn_Area2.BackColor = Color.LightPink;
                else btn_Area2.BackColor = Color.GreenYellow;
                btn_Area1.BackColor = Color.LightPink;

                mf.bnd.bndList[bndSelect].fenceLine_new?.Clear();
                foreach (var item in tempList2)
                {
                    mf.bnd.bndList[bndSelect].fenceLine_new.Add(item);
                }
                btn_Area1.Visible = false;
                area1 = 0;
                label2.Visible = false;
                cboxABLineCurve.Checked = false;
                btnCycleBackwardFirstAB.Visible = false;
                btnCycleForwardFirstAB.Visible = false;
                /*     
                if (cboxABLineCurve.Checked)
                DrawBuiltLinesA();
                else
                DrawBuiltLines();
                */
            }
        }

        private void btnCycleBackwardFirstAB_Click(object sender, EventArgs e)
        {
            btnCycleBackwardFirstAB.Enabled = true;

            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose previous AB line curve ", "                    ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                bndSelectTrim--;
                if (bndSelectTrim < 0) bndSelectTrim = HelpList.Count - 1;
            }
            else
            {
                if (gTemp.Count > 0)
                {
                    if (mf.bndl.idxbndl > 0)
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();
                    DrawBuiltLinesA();
                    nudSetDistancebndl.Value = 0;
                    cboxToolWidthsbndl.Text = "0";
                    indx--;
                    if (indx < 0) indx = gTemp.Count - 1;
                    if (cboxABLineCurve.Checked)
                    {
                        mf.bndl.tracksArrbndl?.Clear();
                        int Valueidx = mf.bndl.idxbndl + 1;
                        if (Valueidx > mf.bndl.tracksArrbndl.Count)
                            Valueidx = 0;
                        label4.Text = (Valueidx + " / " + mf.bndl.tracksArrbndl.Count);
                    }
                    btnDeleteCurvebndl1_Click();
                    btn_Area1.Visible = false;
                    btn_Area2.Visible = false;

                    MakeABLineCurveBorder();

                }
                else
                {
                    indx = -1;
                }

                cboxABBoundary.Checked = false;
                cboxABBoundary.BackColor = Color.Transparent;

                FixLabelsCurveA();
                DrawBuiltLinesA();


            }
        }

        private void btnCycleForwardFirstAB_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose next AB line curve ", "                    ");
                return;
            }
            if (cboxABBoundary.Checked)
            {
                bndSelectTrim++;
                if (bndSelectTrim > HelpList.Count - 1) bndSelectTrim = 0;
            }
            else
            {
                if (gTemp.Count > 0)
                {
                    if (mf.bndl.idxbndl > 0)
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();
                    DrawBuiltLinesA();
                    nudSetDistancebndl.Value = 0;
                    cboxToolWidthsbndl.Text = "0";
                    indx++;
                    if (indx > (gTemp.Count - 1)) indx = 0;
                    if (cboxABLineCurve.Checked)
                    {
                        mf.bndl.tracksArrbndl?.Clear();
                        int Valueidx = mf.bndl.idxbndl + 1;
                        if (Valueidx > mf.bndl.tracksArrbndl.Count)
                            Valueidx = 0;
                        label4.Text = (Valueidx + " / " + mf.bndl.tracksArrbndl.Count);
                    }
                    btnDeleteCurvebndl1_Click();
                    btn_Area1.Visible = false;
                    btn_Area2.Visible = false;

                    MakeABLineCurveBorder();

                }
                else
                {
                    indx = -1;
                }
                cboxABBoundary.Checked = false;
                cboxABBoundary.BackColor = Color.Transparent;

                FixLabelsCurveA();
                DrawBuiltLinesA();
            }
        }

        private void FixLabelsCurveA()
        {
            if (indx > -1 && gTemp.Count > 0)
            {
                cboxABLineCurve.Text = gTemp[indx].name;
                cboxABLineCurve.Enabled = true;
            }
        }

        private void cboxABLineCurve_CheckedChanged(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "choose AB line curve ", "for new boundary");
                cboxABLineCurve.Checked = false;
                return;
            }

            if (cboxABLineCurve.Checked)
            {
                btnALengthbndl.Visible = false;
                btnAShrinkbndl.Visible = false;
                btnBLengthbndl.Visible = false;
                btnBShrinkbndl.Visible = false;
            }
            else
            {
                btnALengthbndl.Visible = true;
                btnAShrinkbndl.Visible = true;
                btnBLengthbndl.Visible = true;
                btnBShrinkbndl.Visible = true;
            }

            if (mf.bndl.idxbndl == -1)
            {
                if (cboxABLineCurve.Checked)
                {
                    if (mf.bndl.idxbndl > 0)
                    {
                        mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl.Clear();
                        mf.bndl.tracksArrbndl.Clear();
                    }
                    DrawBuiltLinesA();
                    nudSetDistancebndl.Value = 0;
                    cboxToolWidthsbndl.Visible = false;
                    btnBndLoopbndl.Visible = false;
                    cboxABLineCurve.BackColor = Color.Lime;
                    btnCycleForwardFirstAB.Visible = true;
                    btnCycleBackwardFirstAB.Visible = true;
                    btnSlice_line.Checked = true;
                    btn_Area1.Visible = false;
                    btn_Area2.Visible = false;
                    MakeABLineCurveBorder();
                    cboxABLineCurve.BackColor = Color.Transparent;

                }
                else
                {
                    btnBndLoopbndl.Visible = true;
                    cboxToolWidthsbndl.Visible = true;
                    cboxABLineCurve.BackColor = Color.Transparent;
                    btnCycleForwardFirstAB.Visible = false;
                    btnCycleBackwardFirstAB.Visible = false;
                    cboxABLineCurve.Text = "AB Line/Curve";
                }
            }
        }

        public double areato = 0, areato1 = 0, areato2 = 0;         // Accumulates area in the loop

        private void lblToolWidth_Click(object sender, EventArgs e)
        {

        }

        private void cboxABBoundary_CheckedChanged(object sender, EventArgs e)
        {
            if (cboxABBoundary.Checked)
            {
                btnSlice_line.Checked = false;
                btnSlice_line.BackColor = Color.Transparent;
                cboxABBoundary.BackColor = Color.LimeGreen;
                btnBndLoopbndl.Visible = true;
                btnCycleBackwardFirstAB.Visible = true;
                btnCycleForwardFirstAB.Visible = true;
                MakeABLineCurveforBorder();
                TouchABPoints?.Clear();
                if (TouchABPoints.Count == 0)
                    for (int ikx = 0; ikx < gTemp.Count; ikx++)
                    {
                        TouchABPoints.Add(new List<vec3>());
                    }

            }
            else cboxABBoundary.BackColor = Color.Transparent;
        }

        private void nudSetDistancebndl_ValueChanged(object sender, EventArgs e)
        {
            
        }

        private void btnZoomPlus_Click(object sender, EventArgs e)
        {
            zoom -= 0.1;
            if (zoom < 0.1) zoom = 0.1;
        }

        private void btnZoomMinus_Click(object sender, EventArgs e)
        {
            zoom += 0.1;
            if (zoom > 2) zoom = 2;
        }

        private void label4_Click(object sender, EventArgs e)
        {

        }

        private void cboxInfoButtons_CheckedChanged(object sender, EventArgs e)
        {
            mf.TimedMessageBox(4000, "when green touch on", "ikone for information");

            if (cboxInfoButtons.Checked) cboxInfoButtons.BackColor = Color.LimeGreen;
            else cboxInfoButtons.BackColor = Color.Transparent;
        }

        public int areato11 = 0, areato22 = 0;         // Accumulates area in the loop
        public double area = 0, area1 = 0, area2 = 0;         // Accumulates area in the loop

        private void nudlessNumericUpDown1_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "volumne  ", "           ");
                return;
            }
            else
            {
                mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
                areato1 = (double)nudlessNumericUpDown1.Value * area1 / 10000;
                areato11 = Convert.ToInt32(areato1);
                lbltotal1.Text = Convert.ToString(areato11);
                btnExitbndl.Focus();
            }
        }

        private void nudlessNumericUpDown2_Click(object sender, EventArgs e)
        {
            if (cboxInfoButtons.Checked)
            {
                mf.TimedMessageBox(2000, "volumne  ", "           ");
                cboxABLineCurve.Checked = false;
                return;
            }
            else
            {
                mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
                areato2 = (double)nudlessNumericUpDown2.Value * area2 / 10000;
                areato22 = Convert.ToInt32(areato2);
                lbltotal2.Text = Convert.ToString(areato22);
                btnExitbndl.Focus();
            }
        }

        // Returns 1 if the lines intersect, otherwis
        public double iE = 0, iN = 0;

        public List<int> crossingsbndl = new List<int>(1);
        private static readonly double crossingpointEast = 0;
        private static readonly double crossingpointNorth = 0;
        public vec3 Crossingpoint = new vec3(crossingpointEast, crossingpointNorth, 0);


        public void ReverseWinding()
        {
            //reverse the boundary
            int cnt = mf.bnd.bndList[bndSelect].fenceLine.Count;
            vec3[] arr = new vec3[cnt];
            cnt--;
            mf.bnd.bndList[bndSelect].fenceLine.CopyTo(arr);
            mf.bnd.bndList[bndSelect].fenceLine.Clear();
            for (int i = cnt; i >= 0; i--)
            {
                arr[i].heading -= Math.PI;
                if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                mf.bnd.bndList[bndSelect].fenceLine.Add(arr[i]);
            }

        }

        bool isClockwise = true;
        //obvious
        public bool CalculateFenceArea()
        {
            int ptCount = mf.bnd.bndList[bndSelect].fenceLine.Count;
            if (ptCount < 1) return false;
            int ptCount1 = tempList1.Count;
            //if (ptCount1 < 1) return false;
            int ptCount2 = tempList2.Count;
            //if (ptCount2 < 1) return false;

            int j = ptCount - 1;  // The last vertex is the 'previous' one to the first
            int j1 = ptCount1 - 1;  // The last vertex is the 'previous' one to the first
            int j2 = ptCount2 - 1;  // The last vertex is the 'previous' one to the first

            area = 0;
            area1 = 0;
            area2 = 0;

            for (int i = 0; i < ptCount; j = i++)
            {
                area += (mf.bnd.bndList[bndSelect].fenceLine[j].easting + mf.bnd.bndList[bndSelect].fenceLine[i].easting) * (mf.bnd.bndList[bndSelect].fenceLine[j].northing - mf.bnd.bndList[bndSelect].fenceLine[i].northing);
            }
            // Area 1
            for (int i = 0; i < ptCount1; j1 = i++)
            {
                area1 += (tempList1[j1].easting + tempList1[i].easting) * (tempList1[j1].northing - tempList1[i].northing);
            }
            // Area 2
            for (int i = 0; i < ptCount2; j2 = i++)
            {
                area2 += (tempList2[j2].easting + tempList2[i].easting) * (tempList2[j2].northing - tempList2[i].northing);
            }

            isClockwise = area >= 0;

            area = Math.Abs(area / 2);
            area1 = Math.Abs(area1 / 2);
            area2 = Math.Abs(area2 / 2);

            //btn_Area1.Visible = true;
            //btn_Area2.Visible = true;

            //mf.bnd.bndList[0].area = area;
            if (mf.isMetric)
            {
                if (lbl_Area.Text == "Area")
                    lbl_Area.Text = Convert.ToString(area / 10000);
                btn_Area1.Text = Convert.ToString(area1 / 10000);
                btn_Area2.Text = Convert.ToString(area2 / 10000);
                label1.Text = "Hektar";
            }
            else
            {
                if (lbl_Area.Text == "Area")
                    lbl_Area.Text = Convert.ToString(area / 10000 * 2.47105);
                btn_Area1.Text = Convert.ToString(area1 / 10000 * 2.47105);
                btn_Area2.Text = Convert.ToString(area2 / 10000 * 2.47105);
                label1.Text = "Acre";
            }
            return isClockwise;
        }
    }
}

