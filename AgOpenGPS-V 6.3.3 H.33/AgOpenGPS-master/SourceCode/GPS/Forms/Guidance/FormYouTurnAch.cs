using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormYouTurnAch : Form
    {
        //access to the main GPS form and all its variables
        private readonly FormGPS mf = null;

        private Point fixPt;

        private bool isA = true;
        private int start = 99999, end = 99999;
        private int bndSelect = 0;

        private bool zoomToggleyt;
        private double zoom = 1, sX = 0, sY = 0;

        public vec3 pint = new vec3(0.0, 1.0, 0.0);

        private bool isLinesVisible = true;

        public FormYouTurnAch(Form callingFormyt)
        {
            //get copy of the calling main form
            mf = callingFormyt as FormGPS;

            InitializeComponent();
            mf.FileLoadYouturnLines();
            mf.CalculateMinMax();
        }

        private void FormYouTurnAch_Load(object sender, EventArgs e)
        {
            mf.Ytl.idxyt = -1;

            mf.FileLoadYouturnLines();
            FixLabelsCurve();

            lblToolWidth.Text = "( " + mf.unitsFtM + " )      Tool: "
                + ((mf.tool.width - mf.tool.overlap) * mf.m2FtOrM).ToString("N1") + mf.unitsFtM + " ";

            mf.bnd.bndList[0].turnLine?.Clear();

            Size = Properties.Settings.Default.setWindow_YouTurnSize;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
            FormYouTurnAche_ResizeEnd(this, e);

            if (!mf.IsOnScreen(Location, Size, 1))
            {
                Top = 0;
                Left = 0;
            }
        }

        private void FormYouTurnLine_FormClosing(object sender, FormClosingEventArgs e)
        {
            mf.FileSaveYouturnLines();

            if (mf.Ytl.tracksArryt.Count > 0)
            {
                mf.Ytl.idxyt = 0;
            }
            else mf.Ytl.idxyt = -1;

            Properties.Settings.Default.setWindow_YouTurnSize = Size;
            Properties.Settings.Default.Save();
        }

        private void FormYouTurnAche_ResizeEnd(object sender, EventArgs e)
        {
            Width = (Height * 4 / 3);

            oglSelfyt.Height = oglSelfyt.Width = Height - 50;

            oglSelfyt.Left = 2;
            oglSelfyt.Top = 2;

            oglSelfyt.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfyt.Width, oglSelfyt.Height);
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);

            tlp1yt.Width = Width - oglSelfyt.Width - 4;
            tlp1yt.Left = oglSelfyt.Width;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
        }

        private void FixLabelsCurve()
        {
            //nudSetDistanceyt.Value = (decimal)((Math.Round((mf.tool.width - mf.tool.overlap) * cboxToolWidthsyt.SelectedIndex, 1)) * mf.m2FtOrM);
        }

        private void oglSelfyt_MouseDown(object sender, MouseEventArgs e)
        {
            Point pt = oglSelfyt.PointToClient(Cursor.Position);

            int wid = oglSelfyt.Width;
            int halfWid = oglSelfyt.Width / 2;
            double scale = (double)wid * 0.903;

            if (cboxIsZoomyt.Checked && !zoomToggleyt)
            {
                sX = ((halfWid - (double)pt.X) / wid) * 1.1;
                sY = ((halfWid - (double)pt.Y) / -wid) * 1.1;
                zoom = 0.1;
                zoomToggleyt = true;
                return;
            }

            zoomToggleyt = false;

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

            mf.bnd.bndList[0].turnLine?.Clear();
            mf.Ytl.idxyt = -1;

            if (isA)
            {
                double minDistA = double.MaxValue;
                start = 99999; end = 99999;

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
                if (rbtnCurveyt.Checked)
                {
                    mf.Ytl.tracksArryt.Add(new CYouTurnPath());
                    mf.Ytl.idxyt = mf.Ytl.tracksArryt.Count - 1;

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

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].a_point = start;
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt?.Clear();

                    if (start < end)
                    {
                        for (int i = start; i <= end; i++)
                        {
                            //calculate the point inside the boundary
                            mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(new vec3(mf.bnd.bndList[bndSelect].fenceLine[i]));

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
                            mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(new vec3(mf.bnd.bndList[bndSelect].fenceLine[i]));

                            if (isLoop && i == 0)
                            {
                                i = mf.bnd.bndList[bndSelect].fenceLine.Count - 1;
                                isLoop = false;
                                end = limit;
                            }
                        }
                    }

                    //who knows which way it actually goes
                    mf.curve.CalculateHeadings(ref mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt);

                    int ptCnt = mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count - 1;

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[ptCnt]);
                        pnt.easting += (Math.Sin(pnt.heading) * i);
                        pnt.northing += (Math.Cos(pnt.heading) * i);
                        mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(pnt);
                    }

                    vec3 stat = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0]);

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(stat);
                        pnt.easting -= (Math.Sin(pnt.heading) * i);
                        pnt.northing -= (Math.Cos(pnt.heading) * i);
                        mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Insert(0, pnt);
                    }

                    //create a name
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].name = mf.Ytl.idxyt.ToString() + " Cu " + DateTime.Now.ToString("mm:ss", CultureInfo.InvariantCulture);

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].moveDistance = 0;

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].mode = (int)TrackMode.Curve;

                    mf.FileSaveYouturnLines();

                    //update the arrays
                    start = 99999; end = 99999;

                    FixLabelsCurve();
                    btnExityt.Focus();
                }
                else if (rbtnLineyt.Checked)
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

                    if (mf.Ytl.idxyt < mf.Ytl.tracksArryt.Count - 1)
                    {
                        mf.Ytl.idxyt++;
                        mf.Ytl.tracksArryt.Insert(mf.Ytl.idxyt, new CYouTurnPath());
                    }
                    else
                    {
                        mf.Ytl.tracksArryt.Add(new CYouTurnPath());
                        mf.Ytl.idxyt = mf.Ytl.tracksArryt.Count - 1;
                    }

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].a_point = start;
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt?.Clear();

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
                        mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(ptC);
                    }

                    int ptCnt = mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count - 1;

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[ptCnt]);
                        pnt.easting += (Math.Sin(pnt.heading) * i);
                        pnt.northing += (Math.Cos(pnt.heading) * i);
                        mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(pnt);
                    }

                    vec3 stat = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0]);

                    for (int i = 1; i < 30; i++)
                    {
                        vec3 pnt = new vec3(stat);
                        pnt.easting -= (Math.Sin(pnt.heading) * i);
                        pnt.northing -= (Math.Cos(pnt.heading) * i);
                        mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Insert(0, pnt);
                    }

                    //create a name
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].name = mf.Ytl.idxyt.ToString() + " AB " + DateTime.Now.ToString("hh:mm:ss", CultureInfo.InvariantCulture);

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].moveDistance = 0;

                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].mode = (int)TrackMode.AB;

                    mf.FileSaveYouturnLines();

                    FixLabelsCurve();
                    start = 99999; end = 99999;
                }

                //mf.bnd.bndList[0].hdLine?.Clear();
                mf.bnd.bndList[0].turnLine?.Clear();
                mf.Ytl.desListyt?.Clear();

                if (mf.Ytl.tracksArryt.Count < 1 || mf.Ytl.idxyt == -1) return;

                double distAway = (double)nudSetDistanceyt.Value * mf.ftOrMtoM;
                mf.Ytl.tracksArryt[mf.Ytl.idxyt].moveDistance += distAway;

                double distSqAway = (distAway * distAway) - 0.01;
                vec3 point;

                int refCount = mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count;
                for (int i = 0; i < refCount; i++)
                {
                    point = new vec3(
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[i].easting - (Math.Sin(glm.PIBy2 + mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[i].heading) * distAway),
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[i].northing - (Math.Cos(glm.PIBy2 + mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[i].heading) * distAway),
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[i].heading);
                    bool Add = true;

                    for (int t = 0; t < refCount; t++)
                    {
                        double dist = ((point.easting - mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[t].easting) * (point.easting - mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[t].easting))
                            + ((point.northing - mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[t].northing) * (point.northing - mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[t].northing));
                        if (dist < distSqAway)
                        {
                            Add = false;
                            break;
                        }
                    }

                    if (Add)
                    {
                        if (mf.Ytl.desListyt.Count > 0)
                        {
                            double dist = ((point.easting - mf.Ytl.desListyt[mf.Ytl.desListyt.Count - 1].easting) * (point.easting - mf.Ytl.desListyt[mf.Ytl.desListyt.Count - 1].easting))
                                + ((point.northing - mf.Ytl.desListyt[mf.Ytl.desListyt.Count - 1].northing) * (point.northing - mf.Ytl.desListyt[mf.Ytl.desListyt.Count - 1].northing));
                            if (dist > 1)
                                mf.Ytl.desListyt.Add(point);
                        }
                        else mf.Ytl.desListyt.Add(point);
                    }
                }

                mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Clear();

                for (int i = 0; i < mf.Ytl.desListyt.Count; i++)
                {
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(new vec3(mf.Ytl.desListyt[i]));
                }

                mf.Ytl.desListyt?.Clear();
            }
        }

        private void oglSelfyt_Paint(object sender, PaintEventArgs e)
        {
            oglSelfyt.MakeCurrent();

            GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.ColorBufferBit);
            GL.LoadIdentity();                  // Reset The View

            //back the camera up
            GL.Translate(0, 0, -mf.maxFieldDistance * zoom);

            //translate to that spot in the world
            GL.Translate(-mf.fieldCenterX + sX * mf.maxFieldDistance, -mf.fieldCenterY + sY * mf.maxFieldDistance, 0);

            GL.LineWidth(2);

            for (int j = 0; j < mf.bnd.bndList.Count; j++)
            {
                if (j == bndSelect)
                    GL.Color3(0.8f, 0.8f, 0.8f);
                else
                    GL.Color3(0.50f, 0.25f, 0.10f);

                GL.Begin(PrimitiveType.Lines);
                for (int i = 0; i < mf.bnd.bndList[j].fenceLine.Count; i++)
                {
                    GL.Vertex3(mf.bnd.bndList[j].fenceLine[i].easting, mf.bnd.bndList[j].fenceLine[i].northing, 0);
                }
                GL.End();
            }

            //the vehicle
            GL.PointSize(8.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
            GL.End();

            //draw the line building graphics
            //if (start != 99999 || end != 99999)
            //draw the actual built lines
            //if (start == 99999 && end == 99999)
            {
                DrawBuiltLines();
            }

            DrawABTouchLine();

            GL.Disable(EnableCap.Blend);

            GL.Flush();
            oglSelfyt.SwapBuffers();
        }

        private void oglSelfyt_Resize(object sender, EventArgs e)
        {
            oglSelfyt.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfyt.Width, oglSelfyt.Height);

            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);
        }

        private void DrawBuiltLines()
        {
            if (isLinesVisible && mf.Ytl.tracksArryt.Count > 0)
            {
                //GL.Enable(EnableCap.LineStipple);
                GL.LineStipple(1, 0x7070);
                GL.PointSize(3);

                for (int i = 0; i < mf.Ytl.tracksArryt.Count; i++)
                {
                    if (mf.Ytl.tracksArryt[i].mode == (int)TrackMode.AB)
                    {
                        GL.Color3(0.973f, 0.9f, 0.10f);
                    }
                    else
                    {
                        GL.Color3(0.3f, 0.99f, 0.20f);
                    }

                    GL.Begin(PrimitiveType.Points);
                    foreach (vec3 item in mf.Ytl.tracksArryt[i].trackPtsyt)
                    {
                        GL.Vertex3(item.easting, item.northing, 0);
                    }
                    GL.End();
                }

                //GL.Disable(EnableCap.LineStipple);

                if (mf.Ytl.idxyt > -1)
                {
                    GL.LineWidth(6);
                    GL.Color3(1.0f, 0.0f, 1.0f);

                    GL.Begin(PrimitiveType.LineStrip);
                    foreach (vec3 item in mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt)
                    {
                        GL.Vertex3(item.easting, item.northing, 0);
                    }
                    GL.End();

                    int cnt = mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count - 1;
                    GL.PointSize(28);
                    GL.Color3(0, 0, 0);
                    GL.Begin(PrimitiveType.Points);
                    GL.Vertex3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0].easting, mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0].northing, 0);
                    GL.Color3(0, 0, 0);
                    GL.Vertex3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[cnt].easting, mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[cnt].northing, 0);
                    GL.End();

                    GL.PointSize(20);
                    GL.Color3(1.0f, 0.7f, 0.35f);
                    GL.Begin(PrimitiveType.Points);
                    GL.Vertex3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0].easting, mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0].northing, 0);
                    GL.Color3(0.6f, 0.75f, 0.99f);
                    GL.Vertex3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[cnt].easting, mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[cnt].northing, 0);
                    GL.End();
                }
            }

            GL.LineWidth(8);
            GL.Color3(0.93f, 0.899f, 0.50f);
            GL.Begin(PrimitiveType.LineStrip);

            for (int i = 0; i < mf.bnd.bndList[0].turnLine.Count; i++)
            {
                GL.Vertex3(mf.bnd.bndList[0].turnLine[i].easting, mf.bnd.bndList[0].turnLine[i].northing, 0);
            }
            GL.End();
        }

        private void DrawABTouchLine()
        {
            GL.Color3(0.65, 0.650, 0.0);
            GL.PointSize(24);
            GL.Begin(PrimitiveType.Points);

            GL.Color3(0, 0, 0);
            if (start != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[start].easting, mf.bnd.bndList[bndSelect].fenceLine[start].northing, 0);
            if (end != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[end].easting, mf.bnd.bndList[bndSelect].fenceLine[end].northing, 0);
            GL.End();

            GL.PointSize(16);
            GL.Begin(PrimitiveType.Points);

            GL.Color3(1.0f, 0.75f, 0.350f);
            if (start != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[start].easting, mf.bnd.bndList[bndSelect].fenceLine[start].northing, 0);

            GL.Color3(0.5f, 0.75f, 1.0f);
            if (end != 99999) GL.Vertex3(mf.bnd.bndList[bndSelect].fenceLine[end].easting, mf.bnd.bndList[bndSelect].fenceLine[end].northing, 0);
            GL.End();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            oglSelfyt.Refresh();
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            mf.FileSaveYouturnLines();
            Close();
        }

        private void nudSetDistanceyt_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            btnExityt.Focus();
        }

        // Returns 1 if the lines intersect, otherwis
        public double iE = 0, iN = 0;

        public List<int> crossingsyt = new List<int>(1);


        private void oglSelfyt_Load(object sender, EventArgs e)
        {
            oglSelfyt.MakeCurrent();
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.ClearColor(0.22f, 0.22f, 0.22f, 1.0f);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
        }

        private void btnBShrinkyt_Click(object sender, EventArgs e)
        {
            if (mf.Ytl.idxyt > -1)
            {
                if (mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count > 8)
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.RemoveRange(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count - 5, 5);
            }
        }

        private void btnBLengthyt_Click(object sender, EventArgs e)
        {
            if (mf.Ytl.idxyt > -1)
            {
                int ptCnt = mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count - 1;

                for (int i = 1; i < 10; i++)
                {
                    vec3 pt = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[ptCnt]);
                    pt.easting += (Math.Sin(pt.heading) * i);
                    pt.northing += (Math.Cos(pt.heading) * i);
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Add(pt);
                }
            }
        }

        private void btnALengthyt_Click(object sender, EventArgs e)
        {
            if (mf.Ytl.idxyt > -1)
            {
                //and the beginning
                vec3 start = new vec3(mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt[0]);

                for (int i = 1; i < 10; i++)
                {
                    vec3 pt = new vec3(start);
                    pt.easting -= (Math.Sin(pt.heading) * i);
                    pt.northing -= (Math.Cos(pt.heading) * i);
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Insert(0, pt);
                }
            }
        }

        private void btnAShrinkyt_Click(object sender, EventArgs e)
        {
            if (mf.Ytl.idxyt > -1)
            {
                if (mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.Count > 8)
                    mf.Ytl.tracksArryt[mf.Ytl.idxyt].trackPtsyt.RemoveRange(0, 5);
            }
        }

        private void btnCancelTouchyt_Click(object sender, EventArgs e)
        {
            //update the arrays
            start = 99999; end = 99999;
            isA = true;
            FixLabelsCurve();
            mf.curve.desList?.Clear();
            zoom = 1;
            sX = 0;
            sY = 0;
            zoomToggleyt = false;
            btnExityt.Focus();
        }

        private void cboxIsZoomyt_CheckedChanged(object sender, EventArgs e)
        {
            zoomToggleyt = false;
        }

        public void BuildTurnLines()
        {
            //update the GUI values for boundaries
            mf.fd.UpdateFieldBoundaryGUIAreas();

            //to fill the list of line points
            vec3 point = new vec3();

            //determine how wide a headland space
            double totalHeadWidth = (double)nudSetDistanceyt.Value;

            //inside boundaries
            for (int j = 0; j < mf.bnd.bndList.Count; j++)
            {
                mf.bnd.bndList[0].turnLine?.Clear();

                int ptCount = mf.bnd.bndList[j].fenceLine.Count;

                for (int i = ptCount - 1; i >= 0; i--)
                {
                    //calculate the point outside the boundary
                    point.easting = mf.bnd.bndList[j].fenceLine[i].easting + (-Math.Sin(glm.PIBy2 + mf.bnd.bndList[j].fenceLine[i].heading) * totalHeadWidth);
                    point.northing = mf.bnd.bndList[j].fenceLine[i].northing + (-Math.Cos(glm.PIBy2 + mf.bnd.bndList[j].fenceLine[i].heading) * totalHeadWidth);
                    point.heading = mf.bnd.bndList[j].fenceLine[i].heading;
                    if (point.heading < -glm.twoPI) point.heading += glm.twoPI;

                    //only add if outside actual field boundary
                    if (j == 0 == mf.bnd.bndList[j].fenceLineEar.IsPointInPolygon(point))
                    {
                        vec3 tPnt = new vec3(point.easting, point.northing, point.heading);
                        mf.bnd.bndList[j].turnLine.Add(tPnt);
                    }
                }
                mf.bnd.bndList[j].FixTurnLine(totalHeadWidth, 2);

                //countExit the reference list of original curve
                int cnt = mf.bnd.bndList[j].turnLine.Count;

                //the temp array
                vec3[] arr = new vec3[cnt];

                for (int s = 0; s < cnt; s++)
                {
                    arr[s] = mf.bnd.bndList[j].turnLine[s];
                }

                double delta = 0;
                mf.bnd.bndList[j].turnLine?.Clear();

                for (int i = 0; i < arr.Length; i++)
                {
                    if (i == 0)
                    {
                        mf.bnd.bndList[j].turnLine.Add(arr[i]);
                        continue;
                    }
                    delta += (arr[i - 1].heading - arr[i].heading);
                    if (Math.Abs(delta) > 0.005)
                    {
                        mf.bnd.bndList[j].turnLine.Add(arr[i]);
                        delta = 0;
                    }
                }

                if (mf.bnd.bndList[j].turnLine.Count > 0)
                {
                    vec3 end = new vec3(mf.bnd.bndList[j].turnLine[0].easting,
                        mf.bnd.bndList[j].turnLine[0].northing, mf.bnd.bndList[j].turnLine[0].heading);
                    mf.bnd.bndList[j].turnLine.Add(end);
                }
            }
        }
        private void btnBndLoopyt_Click(object sender, EventArgs e)
        {
            BNTbndLoopyt();
        }

        public void BNTbndLoopyt()
        {
            //sort the lines
            mf.Ytl.tracksArryt.Sort((p, q) => p.a_point.CompareTo(q.a_point));
            mf.FileSaveYouturnLines();

            mf.Ytl.idxyt = -1;

            //build the turnline
            if (mf.bnd.bndList.Count > -1)
                mf.bnd.bndList[0].turnLine?.Clear();

            int numOfLines = mf.Ytl.tracksArryt.Count;
            int nextLine = 0;
            crossingsyt.Clear();

            int isStart = 0;

            for (int lineNum = 0; lineNum < mf.Ytl.tracksArryt.Count; lineNum++)
            {
                nextLine = lineNum - 1;
                if (nextLine < 0) nextLine = mf.Ytl.tracksArryt.Count - 1;

                if (nextLine == lineNum)
                {
                    mf.TimedMessageBox(2000, "Create Error", "Is there maybe only 1 line?");
                    return;
                }

                for (int i = 0; i < mf.Ytl.tracksArryt[lineNum].trackPtsyt.Count - 2; i++)
                {
                    for (int k = 0; k < mf.Ytl.tracksArryt[nextLine].trackPtsyt.Count - 2; k++)
                    {
                        int res = GetLineIntersectionyt(
                        mf.Ytl.tracksArryt[lineNum].trackPtsyt[i].easting,
                        mf.Ytl.tracksArryt[lineNum].trackPtsyt[i].northing,
                        mf.Ytl.tracksArryt[lineNum].trackPtsyt[i + 1].easting,
                        mf.Ytl.tracksArryt[lineNum].trackPtsyt[i + 1].northing,

                        mf.Ytl.tracksArryt[nextLine].trackPtsyt[k].easting,
                        mf.Ytl.tracksArryt[nextLine].trackPtsyt[k].northing,
                        mf.Ytl.tracksArryt[nextLine].trackPtsyt[k + 1].easting,
                        mf.Ytl.tracksArryt[nextLine].trackPtsyt[k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0) i++;
                            crossingsyt.Add(i);
                            isStart++;
                            if (isStart == 2) goto again;
                            nextLine = lineNum + 1;

                            if (nextLine > mf.Ytl.tracksArryt.Count - 1) nextLine = 0;
                        }
                    }
                }

                again:
                isStart = 0;
            }

            if (crossingsyt.Count != mf.Ytl.tracksArryt.Count * 2)
            {
                mf.TimedMessageBox(2000, "Crosings Error", "Make sure all ends cross only once");
                mf.bnd.bndList[0].turnLine?.Clear();
                return;
            }

            for (int i = 0; i < mf.Ytl.tracksArryt.Count; i++)
            {
                int low = crossingsyt[i * 2];
                int high = crossingsyt[i * 2 + 1];
                for (int k = low; k < high; k++)
                {
                    mf.bnd.bndList[0].turnLine.Add(mf.Ytl.tracksArryt[i].trackPtsyt[k]);
                }
            }

            vec3[] hdArryt;

            if (mf.bnd.bndList[0].turnLine.Count > 0)
            {
                hdArryt = new vec3[mf.bnd.bndList[0].turnLine.Count];
                mf.bnd.bndList[0].turnLine.CopyTo(hdArryt);
                mf.bnd.bndList[0].turnLine?.Clear();
            }
            else
            {
                mf.bnd.bndList[0].turnLine?.Clear();
                return;
            }

            //middle points
            for (int i = 1; i < hdArryt.Length; i++)
            {
                hdArryt[i - 1].heading = Math.Atan2(hdArryt[i - 1].easting - hdArryt[i].easting, hdArryt[i - 1].northing - hdArryt[i].northing);
                if (hdArryt[i].heading < 0) hdArryt[i].heading += glm.twoPI;
            }

            double delta = 0;
            for (int i = 0; i < hdArryt.Length; i++)
            {
                if (i == 0)
                {
                    mf.bnd.bndList[0].turnLine.Add(new vec3(hdArryt[i].easting, hdArryt[i].northing, hdArryt[i].heading));
                    continue;
                }
                delta += (hdArryt[i - 1].heading - hdArryt[i].heading);

                if (Math.Abs(delta) > 0.005)
                {
                    vec3 pt = new vec3(hdArryt[i].easting, hdArryt[i].northing, hdArryt[i].heading);

                    mf.bnd.bndList[0].turnLine.Add(pt);
                    delta = 0;
                }
            }

            mf.FileSaveYouturnLines();
        }

        private void btnDeleteYouturnyt_Click(object sender, EventArgs e)
        {
            start = 99999; end = 99999;
            isA = true;
            mf.Ytl.desListyt?.Clear();
            mf.bnd.bndList[0].turnLine?.Clear();
        }

        private void btnCycleBackwardyt_Click(object sender, EventArgs e)
        {
            mf.bnd.bndList[0].turnLine?.Clear();

            if (mf.Ytl.tracksArryt.Count > 0)
            {
                mf.Ytl.idxyt--;
                if (mf.Ytl.idxyt < 0) mf.Ytl.idxyt = (mf.Ytl.tracksArryt.Count - 1);
            }
            else
            {
                mf.Ytl.idxyt = -1;
            }

            FixLabelsCurve();
        }

        private void btnCycleForwardyt_Click(object sender, EventArgs e)
        {
            mf.bnd.bndList[0].turnLine?.Clear();

            if (mf.Ytl.tracksArryt.Count > 0)
            {
                mf.Ytl.idxyt++;
                if (mf.Ytl.idxyt > (mf.Ytl.tracksArryt.Count - 1)) mf.Ytl.idxyt = 0;
            }
            else
            {
                mf.Ytl.idxyt = -1;
            }

            FixLabelsCurve();
        }

        private void cboxToolWidthsyt_SelectedIndexChanged(object sender, EventArgs e)
        {
            nudSetDistanceyt.Value = (decimal)((Math.Round((mf.tool.width - mf.tool.overlap) * cboxToolWidthsyt.SelectedIndex, 1)) * mf.m2FtOrM);
        }

        private void tlp1yt_Paint(object sender, PaintEventArgs e)
        {

        }

        private void rbtnLineyt_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void btnHeadlandOffyt_Click(object sender, EventArgs e)
        {
            mf.bnd.bndList[0].turnLine?.Clear();
            mf.FileSaveYouturnLines();
            mf.vehicle.isHydLiftOn = false;
            Close();
        }

        private void btnDeleteCurveyt_Click(object sender, EventArgs e)
        {
            //mf.bnd.bndList[0].hdLine?.Clear();

            if (mf.Ytl.tracksArryt.Count > 0 && mf.Ytl.idxyt > -1)
            {
                mf.Ytl.tracksArryt.RemoveAt(mf.Ytl.idxyt);
                mf.Ytl.idxyt--;
            }

            if (mf.hdl.tracksArr.Count > 0)
            {
                if (mf.Ytl.idxyt == -1)
                {
                    mf.Ytl.idxyt++;
                }
            }
            else mf.Ytl.idxyt = -1;

            FixLabelsCurve();
        }

        private void btnExityt_Click(object sender, EventArgs e)
        {
            mf.FileSaveYouturnLines();
            Close();
            //using (var form4 = new FormYouTurn(this))
            //    form4.ShowDialog();
        }

        public int GetLineIntersectionyt(double p0x, double p0y, double p1x, double p1y,
                    double p2x, double p2y, double p3x, double p3y, ref double iEast, ref double iNorth)
        {
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
                    return 1;
                }
            }

            return 0; // No collision
        }

    }
}
