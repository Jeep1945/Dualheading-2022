using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace AgOpenGPS //.Forms.Guidance
{
    public partial class FormContour : Form
    {
        private readonly FormGPS mf = null;

        //private Point fixPt;


        private int maxLineCount;

        // private bool zoomToggle, isCurve = false;
        private int indx = -1;
        private int indxA = -1;
        private int indxB = -1;

        private double zoom = 1, sX = 0, sY = 0;

        public List<CTrk> gTemp = new List<CTrk>();
        public List<vec3> LineA = new List<vec3>();
        public List<vec3> LineB = new List<vec3>();
        public List<vec3> LineC = new List<vec3>();
        public List<vec3> LineD = new List<vec3>();

        public vec3 pint = new vec3(0.0, 1.0, 0.0);

        public List<vec3> pTemp = new List<vec3>();
        public List<vec3> tempListpart5 = new List<vec3>();
        public List<vec3> tempListpart6 = new List<vec3>();
        public List<vec3> tempList1 = new List<vec3>();
        public List<vec3> tempList2 = new List<vec3>();
        public List<vec3> tempLinePointsA = new List<vec3>();
        public List<vec3> tempLinePointsB = new List<vec3>();
        public List<vec3> backupList = new List<vec3>();
        public List<vec3> backupListFence = new List<vec3>();
        public List<vec3> backupListFenceA = new List<vec3>();
        public List<vec3> backupListFenceB = new List<vec3>();
        public List<vec3> ptListPattern = new List<vec3>(128);
        public List<vec3> HeadingTurn = new List<vec3>();
        public List<vec3> ContourPointsABLine = new List<vec3>();
        public List<vec3> ContourPointsABLine1 = new List<vec3>();
        public List<vec3> ContourPointsABLineA1 = new List<vec3>();
        public List<vec3> ContourPointsABLineA2 = new List<vec3>();


        public List<List<vec3>> SideHillTemp = new List<List<vec3>>();
        public List<int> crossingsCont = new List<int>(1);

        //bool TurnABLineandHeading = false;
        bool LineCountCorrect = false;
        //private vec3 ptA = new vec3();
        //private vec3 ptB = new vec3();
        //private vec3 ptC = new vec3();
        //private vec3 ptD = new vec3();
        private double DistanceA1A2;
        private double DistanceA1B1;
        private double DistanceB1B2;
        private double DistanceB2A2;
        private double DistanceA2B2new;
        private double DistanceA1B1new;
        private double MaxDistanceSun;
        private double ChooseDistanceSun;
        public int HowManyLines;
        public int Extentions = 500, MinExtentions = 100;
        public double ToolwidthsunA;
        public double ToolwidthsunB;
        public double[,] DistanceAxBx = new double[3000, 2];
        double DistanceAxBxmin;
        double DistanceAxBxmax;
        public double DistanceAxBxProcend, HeadingExtention = 0;
        public double HeadingExtention1 = 0, HeadingExtention2 = 0;
        private int Case = 1;   // 1: two lines, 2 one line one curve, 3 two curves
        double DisAbefor = -1;
        double DisBbefor = -1;
        double ContourHeading;
        double ContourHeadingA1B1;
        double ContourHeadingA2B2;
        double ContourHeadingAxBx;
        double nudSetDistLineA;
        double nudSetDistLineB;
        double distanceABLineB1B2;


        vec3 CrossPointA1 = new vec3(0, 0, 0);
        vec3 CrossPointA2 = new vec3(0, 0, 0);
        vec3 CrossPointB1 = new vec3(0, 0, 0);
        vec3 CrossPointB2 = new vec3(0, 0, 0);

        //int ix = -1;
        //private bool isZero = false, CrossAABB = false;
        //private string myName = "";

        public double iE = 0, iN = 0;
        private double ToolWithContmax;

        public double CrossingpointEast, CrossingpointNorth;
        public double CrossingpointEast1, CrossingpointNorth1;
        public double CrossingpointEast2, CrossingpointNorth2;


        public double low = 0, high = 1;
        private int iAbefor = -1, iBbefor = -1;

        public FormContour(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;

            InitializeComponent();
            mf.CalculateMinMax();
            //mf.ct.ContourLineList?.Clear();
            mf.FileDeletePatternTracks();

        }

        private void FormContour_Load(object sender, EventArgs e)
        {
            maxLineCount = mf.trk.idx;

            gTemp.Clear();

            foreach (var item in mf.trk.gArr)
            {
                gTemp.Add(new CTrk(item));
            }

            if (gTemp.Count != 0)
            {
                if (mf.trk.idx > -1 && mf.trk.idx <= gTemp.Count)
                {
                    indx = mf.trk.idx;
                }
                else
                    indx = 0;
            }

            indxA = Properties.Settings.Default.set_CounturABLineA;
            indxB = Properties.Settings.Default.set_CounturABLineB;
            Extentions = Properties.Settings.Default.set_Extentions;
            if ((indxA > indx) || (indxB > indx))
            {
                indxA = 0;
                indxB = 1;
            }

            CheckLineCurve();

            //save a backup fenceline
            backupListFence?.Clear();
            backupListFenceA?.Clear();
            backupListFenceB?.Clear();

            foreach (var item in mf.bnd.bndList[0].fenceLine)
            {
                backupListFence.Add(item);
                backupListFenceA.Add(item);
                backupListFenceB.Add(item);
            }

            lblToolWidth.Text = " Toolwith  : "
                + ((mf.tool.width - mf.tool.overlap) * mf.m2FtOrM).ToString("N1") + mf.unitsFtM + " ";

            ToolWithContmax = (double)(mf.tool.width - mf.tool.overlap);// * mf.ftOrMtoM; m2FtOrM

            Size = Properties.Settings.Default.setWindow_ContourlinesSize;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
            FormContour_ResizeEnd(this, e);

            if (!mf.IsOnScreen(Location, Size, 1))
            {
                Top = 0;
                Left = 0;
            }
        }

        private void FormContour_FormClosing(object sender, FormClosingEventArgs e)
        {
            //mf.FileSaveHeadLines();

            if (mf.Cont.tracksArrCont.Count > 0)
            {
                mf.Cont.idxCont = 0;
            }
            else mf.Cont.idxCont = -1;

            Properties.Settings.Default.setWindow_ContourlinesSize = Size;
            Properties.Settings.Default.Save();
        }

        private void timerContour_Tick(object sender, EventArgs e)
        {
            oglSelfCont.Refresh();
        }

        private void FormContour_ResizeEnd(object sender, EventArgs e)
        {
            Width = (Height * 4 / 3);

            oglSelfCont.Height = oglSelfCont.Width = Height - 50;

            oglSelfCont.Left = 2;
            oglSelfCont.Top = 2;

            oglSelfCont.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfCont.Width, oglSelfCont.Height);
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);

            tlp1Cont.Width = Width - oglSelfCont.Width - 4;
            tlp1Cont.Left = oglSelfCont.Width;

            Screen myScreen = Screen.FromControl(this);
            Rectangle area = myScreen.WorkingArea;

            this.Top = (area.Height - this.Height) / 2;
            this.Left = (area.Width - this.Width) / 2;
        }

        private void oglSelfCont_MouseDown(object sender, MouseEventArgs e)
        {
        }

        private void oglSelfCont_Paint(object sender, PaintEventArgs e)
        {
            oglSelfCont.MakeCurrent();

            GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.ColorBufferBit);
            GL.LoadIdentity();                  // Reset The View

            //back the camera up
            GL.Translate(0, 0, -mf.maxFieldDistance * zoom);

            //translate to that spot in the world
            GL.Translate(-mf.fieldCenterX + sX * mf.maxFieldDistance, -mf.fieldCenterY + sY * mf.maxFieldDistance, 0);

            GL.LineWidth(3);


            //  GL.Color3(0.50f, 0.25f, 0.10f);  // brown

            if (mf.bnd.bndList[0].fenceLine.Count > 2)
            {
                //GL.Color4(0.0f, 1.0f, 0.0f, 0.98);      // neon green fenceLine
                GL.Color3(0.8f, 0.8f, 0.8f);              //white
                mf.bnd.bndList[0].fenceLine.DrawPolygon();
            }
            GL.Begin(PrimitiveType.Lines);
            GL.End();

            if (crossingpoint)   // draw the connecting line off the crossingpoints
            {
                CheckCrossingpoints();
                // draws the violett AB buildline
                GL.LineWidth(2);
                GL.Color3(1.0f, 0.8f, 0.2f);   //yellow,orange
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(CrossPointA1.easting, CrossPointA1.northing, 0);
                GL.Vertex3(CrossPointB1.easting, CrossPointB1.northing, 0);
                GL.End();

                GL.LineWidth(2);
                GL.Color3(1.0f, 0.8f, 0.2f);  //yellow,orange
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(CrossPointA2.easting, CrossPointA2.northing, 0);
                GL.Vertex3(CrossPointB2.easting, CrossPointB2.northing, 0);
                GL.End();
            }
            GL.Disable(EnableCap.Blend);
            crossingpoint = true;

            // the Tool
            double sinHR = Math.Sin(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * 0.5; // * mf.m2FtOrM * 0.5;// * mf.ftOrMtoM
            double cosHR = Math.Cos(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * 0.5; // * mf.m2FtOrM * 0.5;
            double sinHL = Math.Sin(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * 0.5; // * mf.m2FtOrM * 0.5;
            double cosHL = Math.Cos(mf.pivotAxlePos.heading + glm.PIBy2) * (mf.tool.width - mf.tool.overlap) * 0.5; // * mf.m2FtOrM * 0.5;

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
            GL.PointSize(10.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
            GL.End();

            GL.PointSize(4.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.00f, 0.0f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
            GL.End();

            // Draw Crossingpoints
            double Texthight, Texthightvalue = 1;
            Texthight = Texthightvalue / 10;
            GL.PointSize(5.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);  // yellow
            GL.Vertex3(CrossPointA1.easting, CrossPointA1.northing, 0.0);
            GL.End();
            string textA1 = " I ";
            GL.Enable(EnableCap.Texture2D);
            GL.BindTexture(TextureTarget.Texture2D, mf.texture[(int)FormGPS.textures.ZoomIn48]);        // Select Our Texture
            mf.font.DrawText(CrossPointA1.easting + Texthightvalue, CrossPointA1.northing + Texthightvalue, textA1, Texthight);
            GL.End();
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.0f, 0.0f);   // red
            GL.Vertex3(CrossPointA2.easting, CrossPointA2.northing, 0.0);
            GL.End();
            string textA2 = " II ";
            GL.Enable(EnableCap.Texture2D);
            mf.font.DrawText(CrossPointA2.easting + Texthightvalue, CrossPointA2.northing + Texthightvalue, textA2, Texthight);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.9f);  // white
            GL.Vertex3(CrossPointB1.easting, CrossPointB1.northing, 0.0);
            GL.End();
            string textB1 = " III ";
            GL.Enable(EnableCap.Texture2D);
            mf.font.DrawText(CrossPointB1.easting + Texthightvalue, CrossPointB1.northing + Texthightvalue, textB1, Texthight);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.0f, 0.90f, 0.0f);   // green
            GL.Vertex3(CrossPointB2.easting, CrossPointB2.northing, 0.0);
            GL.End();
            string textB2 = " IIII ";
            GL.Enable(EnableCap.Texture2D);
            mf.font.DrawText(CrossPointB2.easting + Texthightvalue, CrossPointB2.northing + Texthightvalue, textB2, Texthight);
            GL.Begin(PrimitiveType.Points);
            GL.End();



            //draw the actual built lines
            FixLabelsCurveA();
            FixLabelsCurveB();
            DrawBuiltLinesA();
            DrawBuiltLinesB();

            GL.Flush();
            oglSelfCont.SwapBuffers();
        }

         private void btnSaveContourPattern_Click(object sender, EventArgs e)
        {
            if (mf.ct.ContourLineList.Count > 1)
            {
                mf.trk.gArr.Add(new CTrk());
                mf.trk.gArr[mf.trk.gArr.Count - 1].name = ("&Pa " + 1);
                mf.trk.gArr[mf.trk.gArr.Count - 1].mode = (int)TrackMode.Curve;
                mf.trk.gArr[mf.trk.gArr.Count - 1].curvePts = mf.ct.ContourLineList[0];
            }

            Properties.Settings.Default.set_CounturABLineA = indxA;
            Properties.Settings.Default.set_CounturABLineB = indxB;
            Properties.Settings.Default.set_Extentions = Extentions;
            mf.btnBuildTracks_small.Image = Properties.Resources.Splitlines_On;
            mf.FileSaveTracks();
            Close();
        }

        public void CheckCrossingpoints()
        {
            int res = GetLineIntersectionCont(
                 CrossPointA1.easting,
                 CrossPointA1.northing,
                 CrossPointA2.easting,
                 CrossPointA2.northing,

                 CrossPointB1.easting,
                 CrossPointB1.northing,
                 CrossPointB2.easting,
                 CrossPointB2.northing,
                 ref iE, ref iN);
            if (res == 1)
            {
                crossingpoint = false;
                //mf.TimedMessageBox(3000, "Error", "choose one other line");
                //CrossAABB = true;
                //(CrossPointB1, CrossPointB2) = (CrossPointB2, CrossPointB1);
                return;
            }


            res = GetLineIntersectionCont(
               CrossPointA1.easting,
               CrossPointA1.northing,
               CrossPointB1.easting,
               CrossPointB1.northing,

               CrossPointB2.easting,
               CrossPointB2.northing,
               CrossPointA2.easting,
               CrossPointA2.northing,
               ref iE, ref iN);
            if (res == 1)
            {
                if ((gTemp[indxA].mode == 2) && (2 == gTemp[indxB].mode))
                {
                    (CrossPointB1, CrossPointB2) = (CrossPointB2, CrossPointB1);
                }
                if ((gTemp[indxA].mode == 4) && (2 == gTemp[indxB].mode))
                {
                    (CrossPointB1, CrossPointB2) = (CrossPointB2, CrossPointB1);
                }
                if ((gTemp[indxA].mode == 2) && (4 == gTemp[indxB].mode))
                {
                    (CrossPointA1, CrossPointA2) = (CrossPointA2, CrossPointA1);
                }
                if ((gTemp[indxA].mode == 4) && (4 == gTemp[indxB].mode))
                {
                    (CrossPointA1, CrossPointA2) = (CrossPointA2, CrossPointA1);
                }
            }

            DistanceA1A2 = glm.Distance(CrossPointA1, CrossPointA2);
            DistanceA1B1 = glm.Distance(CrossPointA1, CrossPointB1);
            DistanceB1B2 = glm.Distance(CrossPointB1, CrossPointB2);
            DistanceB2A2 = glm.Distance(CrossPointB2, CrossPointA2);

            if ((LineCountCorrect) && (Case == 1))  // two lines
            {
                if (DistanceA1B1 < DistanceB2A2)
                {
                    DistanceA2B2new = Math.Ceiling(DistanceB2A2 / ToolWithContmax);
                    HowManyLines = (int)DistanceA2B2new;
                    MaxDistanceSun = DistanceB2A2;
                    ToolwidthsunB = MaxDistanceSun / HowManyLines;
                    ToolwidthsunA = DistanceA1B1 / HowManyLines;
                    mf.nudgeDistanceA = ToolwidthsunA;
                    mf.nudgeDistanceB = ToolwidthsunB;
                }
                else
                {
                    DistanceA1B1new = Math.Ceiling(DistanceA1B1 / ToolWithContmax);
                    HowManyLines = (int)DistanceA1B1new;
                    MaxDistanceSun = DistanceA1B1;
                    ToolwidthsunA = MaxDistanceSun / HowManyLines;
                    ToolwidthsunB = DistanceB2A2 / HowManyLines;
                    mf.nudgeDistanceA = ToolwidthsunA;
                    mf.nudgeDistanceB = ToolwidthsunB;
                }

                lblDisA.Text = ((decimal)(DistanceA1A2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDisB.Text = ((decimal)(DistanceB1B2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceA.Text = ((decimal)(DistanceA1B1 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceB.Text = ((decimal)(DistanceB2A2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblCountLines.Text = (HowManyLines + 1).ToString() + " Lines";
                lblCountLinesAB.Text = (HowManyLines + 1).ToString() + " Lines";
                lblMaxWorkwidth.Text = (((MaxDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                ChooseDistanceSun = MaxDistanceSun;
                lblNewWorkWidth.Text = (((ChooseDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
            }

            if ((lblNameCurveA.Text == lblNameCurveB.Text) && (Case == 4))   // one line and one curve
            {
                double nudNotCero;
                if (nudSetDistLineA == 0) nudNotCero = nudSetDistLineB;
                else nudNotCero = (double)nudSetDistLineA;

                DistanceAxBxmax = nudNotCero;
                DistanceAxBxmin = nudNotCero;

            }

            checkLineCurvePoints();

            if ((LineCountCorrect) && ((Case == 2) || (Case == 3)))  // one line and one curve
            {
                DistanceA2B2new = Math.Ceiling(DistanceAxBxmax / ToolWithContmax);
                HowManyLines = (int)DistanceA2B2new;
                MaxDistanceSun = DistanceAxBxmax;
                ToolwidthsunA = DistanceAxBxmin / HowManyLines;
                ToolwidthsunB = MaxDistanceSun / HowManyLines;
                lblCountLines.Text = (HowManyLines + 1).ToString() + " Lines";
                lblMaxWorkwidth.Text = (((MaxDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
            }
            if ((Case == 2) || (Case == 3))  // one line and one curve
            {
                lblDisA.Text = ((decimal)(DistanceA1A2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDisB.Text = ((decimal)(DistanceB1B2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceA.Text = ((decimal)(DistanceAxBxmax * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceB.Text = ((decimal)(DistanceAxBxmin * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblCountLinesAB.Text = (HowManyLines + 1).ToString() + " Lines";
                ChooseDistanceSun = MaxDistanceSun;
                lblNewWorkWidth.Text = (((ChooseDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                DistanceAxBxProcend = (ChooseDistanceSun / HowManyLines) / (DistanceAxBxmax / 100);
            }

            if ((LineCountCorrect) && (Case == 4))  // two parallel curve
            {
                DistanceA2B2new = Math.Ceiling(DistanceAxBxmax / ToolWithContmax);
                HowManyLines = (int)DistanceA2B2new;
                MaxDistanceSun = DistanceAxBxmax;
                ToolwidthsunA = DistanceAxBxmin / HowManyLines;
                ToolwidthsunB = MaxDistanceSun / HowManyLines;
                lblCountLines.Text = (HowManyLines + 1).ToString() + " Lines";
                lblCountLinesAB.Text = ((decimal)((HowManyLines + 1) * mf.ftOrMtoM)).ToString() + " Lines";
                lblMaxWorkwidth.Text = (((MaxDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
            }
            if (Case == 4)  // one line and one curve
            {
                lblDisA.Text = ((decimal)(DistanceA1A2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDisB.Text = ((decimal)(DistanceB1B2 * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceA.Text = ((decimal)(DistanceAxBxmax * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblDistanceB.Text = ((decimal)(DistanceAxBxmin * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                lblCountLinesAB.Text = (HowManyLines + 1).ToString() + " Lines";
                ChooseDistanceSun = MaxDistanceSun;
                lblNewWorkWidth.Text = (((ChooseDistanceSun / HowManyLines) * mf.m2FtOrM)).ToString("N2") + mf.unitsFtM;
                DistanceAxBxProcend = (ChooseDistanceSun / HowManyLines) / (DistanceAxBxmax / 100);
            }
            mf.ct.NewToolWidth = (ChooseDistanceSun / HowManyLines) * mf.m2FtOrM;
            mf.MaxDistanceSunPattern = MaxDistanceSun;
            mf.howManyPathPattern = HowManyLines;
            mf.DistanceA1B1 = DistanceA1B1;
            mf.DistanceA2B2 = DistanceB2A2;

            DrawContourPatternLines();
        }

        private void checkLineCurvePoints()  // AB Line to AB Curve, calculate Dis AB min, Dis AB max
        {
            int FirstCurveCount = tempListpart5.Count;
            int SecondCurveCount = tempListpart6.Count;
            HeadingTurn.Clear();
            int isHesame = 0, WAY = 0;

            if (lblNameCurveA.Text == lblNameCurveB.Text)  // one line and one curve
            {
                return;
            }
            if ((gTemp[indxA].mode == (int)TrackMode.Curve) && (gTemp[indxB].mode == (int)TrackMode.Curve))
            {
                Case = 4;
                return;
            }
            if ((gTemp[indxA].mode == (int)TrackMode.Curve) && (gTemp[indxB].mode == (int)TrackMode.AB))
            {
                distanceABLineB1B2 = DistanceB1B2;
                distanceABLineB1B2 /= FirstCurveCount;
                Case = 2;
                tempLinePointsB.Clear();

                // have to check if heading is the same
                if (tempListpart6.Count > 0)
                    if ((Math.Abs(tempListpart6[0].heading - gTemp[indxA].heading) < (glm.PIBy2)) || (Math.Abs(tempListpart6[0].heading - gTemp[indxA].heading) > (glm.PIBy2 * 3)))
                        isHesame = 0;
                    else isHesame = tempListpart6.Count - 1;

                if (tempListpart6.Count > 2)
                {
                    vec3 pntB = new vec3(tempListpart6[isHesame]);
                    for (int i = 0; i < FirstCurveCount; i++)
                    {
                        if (isHesame == 0)
                        {
                            pntB.easting += (Math.Sin(tempListpart6[0].heading) * distanceABLineB1B2);
                            pntB.northing += (Math.Cos(tempListpart6[0].heading) * distanceABLineB1B2);
                            pntB.heading = tempListpart6[0].heading;
                            tempLinePointsB.Add(pntB);
                            WAY = 1;
                        }
                        else
                        {
                            pntB.easting += (Math.Sin(tempListpart6[0].heading + Math.PI) * distanceABLineB1B2);
                            pntB.northing += (Math.Cos(tempListpart6[0].heading + Math.PI) * distanceABLineB1B2);
                            pntB.heading = tempListpart6[0].heading + Math.PI;
                            tempLinePointsB.Add(pntB);
                            WAY = 2;
                        }
                    }
                }

                //calculate the min and max distance
                DistanceAxBxmin = 0;
                DistanceAxBxmax = 0;
                if (tempLinePointsB.Count > 0)
                {
                    if (tempListpart5.Count == tempLinePointsB.Count)
                    {
                        for (int iDis = 0; iDis < FirstCurveCount; iDis++)
                        {
                            DistanceAxBx[iDis, 0] = glm.Distance(tempLinePointsB[iDis], tempListpart5[iDis]);
                            ContourHeadingAxBx = Math.Atan2(tempLinePointsB[iDis].easting - tempListpart5[iDis].easting, tempLinePointsB[iDis].northing - tempListpart5[iDis].northing);
                            ContourHeadingAxBx = glm.Radiant0To2Pi(ContourHeadingAxBx);
                            DistanceAxBx[iDis, 1] = ContourHeadingAxBx;
                            //Console.WriteLine("iDis WAY  : " + iDis + " : " + WAY);
                            //Console.WriteLine("isHesame  : " + isHesame);
                            //Console.WriteLine("T6  indxA  : " + tempListpart6[0].heading + "  " + gTemp[indxA].heading);
                            //Console.WriteLine("tempLinePointsB[iDis] : " + tempLinePointsB[iDis].easting + "  " + tempLinePointsB[iDis].northing);
                            //Console.WriteLine("tempListpart5  [iDis] : " + tempListpart5[iDis].easting + "  " + tempListpart5[iDis].northing);
                            //Console.WriteLine("DistanceAxBx[iDis,0]  : " + DistanceAxBx[iDis, 0]);
                            //Console.WriteLine("DistanceHeading[iDis,1]  : " + DistanceAxBx[iDis, 1]);
                            if ((DistanceAxBxmin == 0) || (DistanceAxBxmin > DistanceAxBx[iDis, 0]))
                                DistanceAxBxmin = DistanceAxBx[iDis, 0];
                            if (DistanceAxBxmax < DistanceAxBx[iDis, 0])
                                DistanceAxBxmax = DistanceAxBx[iDis, 0];
                        }
                    }
                }
            }
            /*
            if ((gTemp[indxA].mode == (int)TrackMode.AB) && (gTemp[indxB].mode == (int)TrackMode.Curve))
            {
                double distanceABLineA1A2 = DistanceA1A2;
                distanceABLineA1A2 /= SecondCurveCount;
                Case = 2;
                tempLinePointsA.Clear();

                // have to check if heading is the same
                isHesame = 0;
                if (tempListpart5.Count > 0)
                    if ((Math.Abs(tempListpart5[0].heading - gTemp[indxB].heading) < (glm.PIBy2)) || (Math.Abs(tempListpart5[0].heading - gTemp[indxB].heading) > (glm.PIBy2 * 3)))
                        isHesame = 0;
                    else isHesame = tempListpart5.Count - 1;

                if (tempListpart5.Count > 2)
                {
                    vec3 pntA = new vec3(tempListpart5[isHesame]);
                    for (int i = 0; i < SecondCurveCount; i++)
                    {
                        if (isHesame == 0)
                        {
                            pntA.easting += (Math.Sin(tempListpart5[0].heading) * distanceABLineA1A2);
                            pntA.northing += (Math.Cos(tempListpart5[0].heading) * distanceABLineA1A2);
                            pntA.heading = tempListpart5[0].heading;
                            tempLinePointsA.Add(pntA);
                            WAY = 3;
                        }
                        else
                        {
                            pntA.easting += (Math.Sin(tempListpart5[0].heading + Math.PI) * distanceABLineA1A2);
                            pntA.northing += (Math.Cos(tempListpart5[0].heading + Math.PI) * distanceABLineA1A2);
                            pntA.heading = tempListpart5[0].heading + Math.PI;
                            tempLinePointsA.Add(pntA);
                            WAY = 4;
                        }
                    }
                    //calculate the min and max distance
                    DistanceAxBxmin = 0;
                    DistanceAxBxmax = 0;
                    if (tempLinePointsA.Count > 0)
                    {
                        if (tempListpart6.Count == tempLinePointsA.Count)
                        {
                            for (int iDis = 0; iDis < SecondCurveCount; iDis++)
                            {
                                DistanceAxBx[iDis, 0] = glm.Distance(tempLinePointsA[iDis], tempListpart6[iDis]);
                                ContourHeadingAxBx = Math.Atan2(tempLinePointsA[iDis].easting - tempListpart6[iDis].easting, tempLinePointsA[iDis].northing - tempListpart6[iDis].northing);
                                ContourHeadingAxBx += glm.PIBy2;
                                ContourHeadingAxBx = glm.twoPI - ContourHeadingAxBx;
                                if (ContourHeadingAxBx < 0) ContourHeadingAxBx += glm.twoPI;
                                else if (ContourHeadingAxBx > glm.twoPI) ContourHeadingAxBx -= glm.twoPI;
                                DistanceAxBx[iDis, 1] = ContourHeadingAxBx;
                                //Console.WriteLine("iDis WAY  : " + iDis + " : " + WAY);
                                //Console.WriteLine("isHesame  : " + isHesame);
                                //Console.WriteLine("T5  indxB  : " + tempListpart5[0].heading + "  " + gTemp[indxB].heading);
                                //Console.WriteLine("tempLinePointsA[iDis] : " + tempLinePointsA[iDis].easting + "  " + tempLinePointsA[iDis].northing);
                                //Console.WriteLine("tempListpart6  [iDis] : " + tempListpart6[iDis].easting + "  " + tempListpart6[iDis].northing);
                                //Console.WriteLine("DistanceAxBx[iDis,0]  : " + DistanceAxBx[iDis, 0]);

                                if ((DistanceAxBxmin == 0) || (DistanceAxBxmin > DistanceAxBx[iDis, 0]))
                                    DistanceAxBxmin = DistanceAxBx[iDis, 0];
                                if (DistanceAxBxmax < DistanceAxBx[iDis, 0])
                                    DistanceAxBxmax = DistanceAxBx[iDis, 0];
                            }
                        }
                    }
                }
            }*/
        }

        private void DrawBuiltLinesA()
        {
            GL.LineStipple(1, 0x0707);

            int i;
            i = indxA;

            //AB Lines
            if (gTemp[i].mode == (int)TrackMode.AB)
            {
                if (nudSetDistanceLineA.Value == 0)
                {
                    GL.LineWidth(4);
                    GL.Color3(0.30f, 0.7f, 0.30f);      // green
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                }
                else
                {
                    GL.Enable(EnableCap.LineStipple);
                    GL.LineStipple(1, 0x0F00);
                    GL.Color3(0.30f, 0.7f, 0.30f);      // green
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.End();
                    GL.Disable(EnableCap.LineStipple);
                    GL.End();
                }

                if (nudSetDistanceLineA.Value != 0)
                {
                    GL.LineWidth(4);
                    GL.Color3(0.30f, 0.7f, 0.30f);      // red
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * nudSetDistLineA, gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * nudSetDistLineA, 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * nudSetDistLineA, gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * nudSetDistLineA, 0);
                    GL.End();

                    // draws the violett AB buildline
                    GL.LineWidth(4);               // cut line
                    GL.Color3(1.0f, 0.0f, 1.0f);   //violett  
                    GL.Begin(PrimitiveType.Lines);
                    //GL.Vertex3(CrossPointA1.easting, CrossPointA1.northing, 0);
                    //GL.Vertex3(CrossPointA2.easting, CrossPointA2.northing, 0);
                }

                GL.End();
                GL.Disable(EnableCap.LineStipple);
            }

            else if (gTemp[i].mode == (int)TrackMode.Curve)
            {
                GL.LineWidth(4);                    // for Draw of AB Curve
                GL.Color3(0.30f, 0.7f, 0.30f);      // green
                GL.Disable(EnableCap.LineStipple);
                GL.Begin(PrimitiveType.LineStrip);

                foreach (vec3 pts in tempListpart5)
                {
                    GL.Vertex3(pts.easting, pts.northing, 0);   // Draw AB Curve tempListpart5
                }
                GL.End();
                GL.Disable(EnableCap.LineStipple);
                GL.End();
            }
            CrossingpointsA();
        }

        private void DrawBuiltLinesB()
        {
            GL.LineStipple(1, 0x0707);

            int i;
            i = indxB;

            //AB Lines
            if (gTemp[i].mode == (int)TrackMode.AB)
            {
                if (nudSetDistanceLineB.Value == 0)
                {
                    GL.LineWidth(4);
                    GL.Color3(2.0f, 0.20f, 0.20f);      // red
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                }
                else
                {
                    GL.Enable(EnableCap.LineStipple);
                    GL.LineStipple(1, 0x0F00);
                    GL.Color3(2.0f, 0.20f, 0.20f);      // red
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength), gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength), 0);
                    GL.End();
                    GL.Disable(EnableCap.LineStipple);
                    GL.End();
                }

                if (nudSetDistanceLineB.Value != 0)
                {
                    GL.LineWidth(2);
                    GL.Color3(2.0f, 0.20f, 0.20f);      // red
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(gTemp[i].ptA.easting - (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * nudSetDistLineB, gTemp[i].ptA.northing - (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * nudSetDistLineB, 0);
                    GL.Vertex3(gTemp[i].ptB.easting + (Math.Sin(gTemp[i].heading) * mf.ABLine.abLength) + Math.Cos(gTemp[i].heading) * nudSetDistLineB, gTemp[i].ptB.northing + (Math.Cos(gTemp[i].heading) * mf.ABLine.abLength) - Math.Sin(gTemp[i].heading) * nudSetDistLineB, 0);
                    GL.End();

                    // draws the violett AB buildline
                    GL.LineWidth(4);               // cut line
                    GL.Color3(1.0f, 0.1f, 1.0f);   //violett  
                    GL.Begin(PrimitiveType.Lines);
                    GL.Vertex3(CrossPointB1.easting, CrossPointB1.northing, 0);
                    GL.Vertex3(CrossPointB2.easting, CrossPointB2.northing, 0);
                }
                GL.End();
                GL.Disable(EnableCap.LineStipple);

            }

            else if (gTemp[i].mode == (int)TrackMode.Curve)
            {
                GL.LineWidth(2);                    // for Draw of AB Curve
                GL.Color3(2.0f, 0.20f, 0.20f);      // red
                GL.Disable(EnableCap.LineStipple);
                GL.Begin(PrimitiveType.LineStrip);

                foreach (vec3 pts in tempListpart6)
                {
                    GL.Vertex3(pts.easting, pts.northing, 0);   // Draw AB Curve
                }

                GL.End();
                GL.Disable(EnableCap.LineStipple);
                GL.End();
                // draws the violett AB buildline
                GL.LineWidth(4);               // cut line
                GL.Color3(1.0f, 0.0f, 1.0f);   //violett  tempListpart5
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(CrossPointB1.easting, CrossPointB1.northing, 0);
                GL.Vertex3(CrossPointB2.easting, CrossPointB2.northing, 0);
                GL.End();
            }
            CrossingpointsB();
        }

        private void DrawContourPatternLines()
        {
            if ((DistanceA1B1 < 20) || (DistanceB2A2 < 20))
                return;

            double A1A2Distance = 0;
            double head = HowManyLines / 2;

            if (Extentions < MinExtentions) Extentions = MinExtentions;
            checkLineCurvePoints();

            mf.ct.ContourLineList?.Clear();

            // Headingdirection from line A to line B
            //Console.WriteLine("      : ");
            ContourHeadingA1B1 = Math.Atan2(CrossPointB1.easting - CrossPointA1.easting, CrossPointB1.northing - CrossPointA1.northing);
            ContourHeadingA2B2 = Math.Atan2(CrossPointB2.easting - CrossPointA2.easting, CrossPointB2.northing - CrossPointA2.northing);
            //Console.WriteLine("ContourHeadingA1B1 0: " + glm.toDegrees(ContourHeadingA1B1));
            //Console.WriteLine("ContourHeadingA2B2 0: " + glm.toDegrees(ContourHeadingA2B2));
            //Console.WriteLine("ContourDegreesA1B1 1: " + glm.toDegrees(ContourDegreesA1B1));
            //Console.WriteLine("ContourDegreesA2B2 1: " + glm.toDegrees(ContourDegreesA2B2));

            // Line of choose from list
            if ((gTemp[indxA].mode == (int)TrackMode.AB) && (gTemp[indxB].mode == (int)TrackMode.AB)) // Line of choise from list
            {

                for (int iCL = 0; iCL < HowManyLines + 1; iCL++)
                {
                    mf.ct.ContourLineList.Add(new List<vec3>());
                    double Pointdiff = 0;
                    //ptListPattern.Clear();
                    vec3 ContourPointsABLineA1 = new vec3(0.0, 0.0, 0.0);
                    vec3 ContourPointsABLineA2 = new vec3(0.0, 0.0, 0.0);
                    // points at the connectinglines
                    ContourPointsABLineA1.easting = CrossPointA1.easting + Math.Sin(ContourHeadingA1B1) * (double)ToolwidthsunA * iCL;// * mf.m2FtOrM;
                    ContourPointsABLineA1.northing = CrossPointA1.northing + Math.Cos(ContourHeadingA1B1) * (double)ToolwidthsunA * iCL;// * mf.m2FtOrM;
                    ContourPointsABLineA2.easting = CrossPointA2.easting + Math.Sin(ContourHeadingA2B2) * (double)ToolwidthsunB * iCL;// * mf.m2FtOrM;
                    ContourPointsABLineA2.northing = CrossPointA2.northing + Math.Cos(ContourHeadingA2B2) * (double)ToolwidthsunB * iCL;// * mf.m2FtOrM;
                    A1A2Distance = glm.Distance(ContourPointsABLineA1, ContourPointsABLineA2);
                    // heading of the new AB line from A1B1 line to A2B2 line
                    ContourHeading = glm.Radiant0To2Pi(Math.Atan2(ContourPointsABLineA2.easting - ContourPointsABLineA1.easting, ContourPointsABLineA2.northing - ContourPointsABLineA1.northing));
                    ContourPointsABLineA1.heading = ContourHeading;
                    ContourPointsABLineA2.heading = ContourHeading;
                    mf.ct.ContourLineList[iCL].Add(ContourPointsABLineA1);

                    vec3 ContourPointsABLine2 = new vec3(0.0, 0.0, 0.0);
                    ContourPointsABLine2.easting = mf.ct.ContourLineList[iCL][0].easting;
                    ContourPointsABLine2.northing = mf.ct.ContourLineList[iCL][0].northing;

                    while (Pointdiff < A1A2Distance)  // make lines between Crossingpoints A1-A2 and B1-B2
                    {
                        Pointdiff += 1.3;
                        ContourPointsABLine2.easting += (Math.Sin(ContourHeading) * 1.3); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine2.northing += (Math.Cos(ContourHeading) * 1.3); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine2.heading = ContourHeading;
                        mf.ct.ContourLineList[iCL].Add(ContourPointsABLine2);
                    }

                    if (Math.Ceiling(head) == iCL)
                    {
                        HeadingExtention = glm.Radiant0To2Pi(mf.ct.ContourLineList[iCL][0].heading);
                    }
                    //Console.WriteLine(" : ");
                    //Console.WriteLine("HeadingoutoffA1B1Low        : " + glm.toDegrees(HeadingoutoffA1B1Low));
                    //Console.WriteLine("HeadingoutoffA1B1High       : " + glm.toDegrees(HeadingoutoffA1B1High));
                    //Console.WriteLine("HeadingExtention                  : " + glm.toDegrees(HeadingExtention));
                    //Console.WriteLine("ContourHeading    : " + glm.toDegrees(ContourHeading));
                    //Console.WriteLine(" : ");
                    //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][0].easting + "  " + mf.ct.ContourLineList[iCL][0].northing + "  " + mf.ct.ContourLineList[iCL][0].heading);
                    //Console.WriteLine("Line : " + iCL + " [1] " + mf.ct.ContourLineList[iCL][1].easting + "  " + mf.ct.ContourLineList[iCL][1].northing + "  " + mf.ct.ContourLineList[iCL][1].heading);
                    //Console.WriteLine("Line : " + iCL + " [2] " + mf.ct.ContourLineList[iCL][2].easting + "  " + mf.ct.ContourLineList[iCL][2].northing + "  " + mf.ct.ContourLineList[iCL][2].heading);
                    //Console.WriteLine(" : ");
                }
                // add the extention lines at beginn and end
                for (int iCL = 0; iCL < mf.ct.ContourLineList.Count; iCL++)
                {
                    vec3 ContourPointsABLine1 = new vec3(0.0, 0.0, 0.0);
                    vec3 ContourPointsABLine2 = new vec3(0.0, 0.0, 0.0);
                    ContourPointsABLine1.easting = mf.ct.ContourLineList[iCL][0].easting;
                    ContourPointsABLine1.northing = mf.ct.ContourLineList[iCL][0].northing;

                    int CountExtentPoints = (int)(Extentions / 1.3);  // A1A2Distance);
                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        ContourPointsABLine1.easting -= (Math.Sin(HeadingExtention) * 1.3); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine1.northing -= (Math.Cos(HeadingExtention) * 1.3); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine1.heading = HeadingExtention;
                        mf.ct.ContourLineList[iCL].Insert(0, ContourPointsABLine1);
                    }
                    ContourPointsABLine2.easting = mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].easting;
                    ContourPointsABLine2.northing = mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].northing;

                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        ContourPointsABLine2.easting += (Math.Sin(HeadingExtention) * 1.3); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine2.northing += (Math.Cos(HeadingExtention) * 1.3); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        ContourPointsABLine2.heading = HeadingExtention;
                        mf.ct.ContourLineList[iCL].Add(ContourPointsABLine2);
                    }

                    if (iCL < -1)
                    {
                        int mpoints1 = mf.ct.ContourLineList[iCL].Count;

                        Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][0].easting + "  " + mf.ct.ContourLineList[iCL][0].northing + "  " + mf.ct.ContourLineList[iCL][0].heading);
                        Console.WriteLine("Line : " + iCL + " [1] " + mf.ct.ContourLineList[iCL][1].easting + "  " + mf.ct.ContourLineList[iCL][1].northing + "  " + mf.ct.ContourLineList[iCL][1].heading);
                        Console.WriteLine("Line : " + iCL + " [2] " + mf.ct.ContourLineList[iCL][2].easting + "  " + mf.ct.ContourLineList[iCL][2].northing + "  " + mf.ct.ContourLineList[iCL][2].heading);
                        Console.WriteLine("Line : " + iCL + " [4] " + mf.ct.ContourLineList[iCL][mpoints1 - 5].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 5].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 5].heading);
                        Console.WriteLine("Line : " + iCL + " [3] " + mf.ct.ContourLineList[iCL][mpoints1 - 4].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 4].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 4].heading);
                        Console.WriteLine("Line : " + iCL + " [2] " + mf.ct.ContourLineList[iCL][mpoints1 - 3].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 3].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 3].heading);
                        Console.WriteLine("Line : " + iCL + " [1] " + mf.ct.ContourLineList[iCL][mpoints1 - 2].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 2].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 2].heading);
                        Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 1].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 1].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 1].heading);
                    }

                }
                // draws the violett AB Patternline
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue 
                GL.Begin(PrimitiveType.Lines);
                for (int iCL = 0; iCL < mf.ct.ContourLineList.Count; iCL++)
                {
                    //GL.Vertex3(mf.ct.ContourLineList[iCL][1].easting, mf.ct.ContourLineList[iCL][1].northing, 0);
                    //GL.Vertex3(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].northing, 0);

                }
                GL.End();
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue 
                GL.Begin(PrimitiveType.Lines);
                for (int iCL = 0; iCL < mf.ct.ContourLineList.Count; iCL++)
                {
                    GL.Vertex3(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].northing, 0);
                    GL.Vertex3(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].northing, 0);

                }
                GL.End();
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue 
                GL.Disable(EnableCap.LineStipple);
                GL.Begin(PrimitiveType.Lines);
                for (int iCL = 0; iCL < mf.ct.ContourLineList.Count; iCL++)
                {
                    GL.Vertex3(mf.ct.ContourLineList[iCL][0].easting, mf.ct.ContourLineList[iCL][0].northing, 0);
                    GL.Vertex3(mf.ct.ContourLineList[iCL][1].easting, mf.ct.ContourLineList[iCL][1].northing, 0);

                }
                GL.Disable(EnableCap.LineStipple);

                GL.End();

            }

            if ((gTemp[indxA].mode == (int)TrackMode.Curve) || (gTemp[indxB].mode == (int)TrackMode.Curve)) // Line of choise from list
            {
                vec3 ContourPointsABLineA3 = new vec3(0.0, 0.0, 0.0);
                vec3 ContourPointsABLineA1 = new vec3(0.0, 0.0, 0.0);
                vec3 ContourPointsABLineA2 = new vec3(0.0, 0.0, 0.0);

                for (int iCL = 0; iCL < HowManyLines + 1; iCL++)
                {

                    mf.ct.ContourLineList.Add(new List<vec3>());

                    //ptListPattern.Clear();

                    for (int icurpoi = 0; icurpoi < tempListpart5.Count - 1; icurpoi++)
                    {
                        // points at the connectinglines
                        ContourPointsABLineA1.easting = tempListpart5[icurpoi].easting + Math.Sin((double)DistanceAxBx[icurpoi, 1]) * (double)DistanceAxBx[icurpoi, 0] * DistanceAxBxProcend / 100 * iCL;// * mf.m2FtOrM;
                        ContourPointsABLineA1.northing = tempListpart5[icurpoi].northing + Math.Cos((double)DistanceAxBx[icurpoi, 1]) * (double)DistanceAxBx[icurpoi, 0] * DistanceAxBxProcend / 100 * iCL;// * mf.m2FtOrM;
                        ContourPointsABLineA2.easting = tempListpart5[icurpoi + 1].easting + Math.Sin((double)DistanceAxBx[icurpoi + 1, 1]) * (double)DistanceAxBx[icurpoi + 1, 0] * DistanceAxBxProcend / 100 * iCL;// * mf.m2FtOrM;
                        ContourPointsABLineA2.northing = tempListpart5[icurpoi + 1].northing + Math.Cos((double)DistanceAxBx[icurpoi + 1, 1]) * (double)DistanceAxBx[icurpoi + 1, 0] * DistanceAxBxProcend / 100 * iCL;// * mf.m2FtOrM;
                        // heading of the new AB line
                        ContourHeading = Math.Atan2(ContourPointsABLineA2.easting - ContourPointsABLineA1.easting, ContourPointsABLineA2.northing - ContourPointsABLineA1.northing);
                        ContourHeading = glm.Radiant0To2Pi(ContourHeading);

                        ContourPointsABLineA1.heading = ContourHeading;
                        mf.ct.ContourLineList[iCL].Add(ContourPointsABLineA1);
                    }
                    mf.ct.ContourLineList[iCL].RemoveAt(0);

                    A1A2Distance = glm.Distance(mf.ct.ContourLineList[iCL][1], mf.ct.ContourLineList[iCL][2]);
                    //Console.WriteLine(" A1A2Distance: " + iCL + " iCL " + A1A2Distance); 
                    if (Math.Ceiling(head) == iCL)
                    {
                        HeadingExtention1 = glm.Radiant0To2Pi(mf.ct.ContourLineList[iCL][0].heading);
                        HeadingExtention2 = glm.Radiant0To2Pi(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].heading);
                    }
                    //Console.WriteLine(" : ");
                    //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][0].easting + "  " + mf.ct.ContourLineList[iCL][0].northing + "  " + mf.ct.ContourLineList[iCL][0].heading);
                    //Console.WriteLine("Line : " + iCL + " [1] " + mf.ct.ContourLineList[iCL][1].easting + "  " + mf.ct.ContourLineList[iCL][1].northing + "  " + mf.ct.ContourLineList[iCL][1].heading);
                    //Console.WriteLine("Line : " + iCL + " [2] " + mf.ct.ContourLineList[iCL][2].easting + "  " + mf.ct.ContourLineList[iCL][2].northing + "  " + mf.ct.ContourLineList[iCL][2].heading);
                    //Console.WriteLine(" : ");
                    //Console.WriteLine("HeadingoutoffA1B1Low        : " + glm.toDegrees(HeadingoutoffA1B1Low));
                    //Console.WriteLine("HeadingoutoffA1B1High       : " + glm.toDegrees(HeadingoutoffA1B1High));
                    //Console.WriteLine("Headingsum                  : " + glm.toDegrees(HeadingExtention));
                    //Console.WriteLine("ContourHeading    : " + glm.toDegrees(ContourHeading) + "   HeadingExtention    : " + glm.toDegrees(HeadingExtention));
                    //Console.WriteLine("HeadingExtention1    : " + glm.toDegrees(HeadingExtention1));
                    //Console.WriteLine("HeadingExtention2    : " + glm.toDegrees(HeadingExtention2));
                }
                for (int iCL = 0; iCL < mf.ct.ContourLineList.Count; iCL++)
                {
                    vec3 ContourPointsABLine = new vec3(0.0, 0.0, 0.0);
                    ContourPointsABLine.easting = mf.ct.ContourLineList[iCL][0].easting;
                    ContourPointsABLine.northing = mf.ct.ContourLineList[iCL][0].northing;

                    int CountExtentPoints = (int)(Extentions / A1A2Distance);
                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        ContourPointsABLine.easting -= Math.Sin(HeadingExtention1) * A1A2Distance;// * mf.m2FtOrM;
                        ContourPointsABLine.northing -= Math.Cos(HeadingExtention1) * A1A2Distance;// * mf.m2FtOrM;
                        ContourPointsABLine.heading = HeadingExtention1;
                        mf.ct.ContourLineList[iCL].Insert(0, ContourPointsABLine);
                        //if (iextent == 1)
                        //mf.ct.ContourLineList[iCL].RemoveAt(1);
                    }
                    for (int iextent = 1; iextent < CountExtentPoints; iextent++)
                    {
                        int mpoints = mf.ct.ContourLineList[iCL].Count;
                        ContourPointsABLineA2.easting = mf.ct.ContourLineList[iCL][mpoints - 1].easting + Math.Sin(HeadingExtention2) * A1A2Distance;// * mf.m2FtOrM;
                        ContourPointsABLineA2.northing = mf.ct.ContourLineList[iCL][mpoints - 1].northing + Math.Cos(HeadingExtention2) * A1A2Distance;// * mf.m2FtOrM;
                        ContourPointsABLineA2.heading = HeadingExtention2;
                        mf.ct.ContourLineList[iCL].Add(ContourPointsABLineA2);
                    }
                    if (iCL < 2)
                    {
                        int mpoints1 = mf.ct.ContourLineList[iCL].Count;
                        //Console.WriteLine("mpoints1   " + mpoints1);
                        //Console.WriteLine("HeadingExtention1    : " + glm.toDegrees(HeadingExtention1));
                        //Console.WriteLine("HeadingoutoffA1B1Low1        : " + glm.toDegrees(HeadingoutoffA1B1Low1));
                        //Console.WriteLine("HeadingoutoffA1B1High1       : " + glm.toDegrees(HeadingoutoffA1B1High1));
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][0].easting + "  " + mf.ct.ContourLineList[iCL][0].northing + "  " + mf.ct.ContourLineList[iCL][0].heading);
                        //Console.WriteLine("Line : " + iCL + " [1] " + mf.ct.ContourLineList[iCL][1].easting + "  " + mf.ct.ContourLineList[iCL][1].northing + "  " + mf.ct.ContourLineList[iCL][1].heading);
                        //Console.WriteLine("Line : " + iCL + " [2] " + mf.ct.ContourLineList[iCL][2].easting + "  " + mf.ct.ContourLineList[iCL][2].northing + "  " + mf.ct.ContourLineList[iCL][2].heading);
                        //Console.WriteLine("Line : " + iCL + " [3] " + mf.ct.ContourLineList[iCL][3].easting + "  " + mf.ct.ContourLineList[iCL][3].northing + "  " + mf.ct.ContourLineList[iCL][0].heading);
                        //Console.WriteLine("Line : " + iCL + " [4] " + mf.ct.ContourLineList[iCL][4].easting + "  " + mf.ct.ContourLineList[iCL][4].northing + "  " + mf.ct.ContourLineList[iCL][1].heading);
                        //Console.WriteLine("Line : " + iCL + " [5] " + mf.ct.ContourLineList[iCL][5].easting + "  " + mf.ct.ContourLineList[iCL][5].northing + "  " + mf.ct.ContourLineList[iCL][2].heading);
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 5].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 5].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 5].heading);
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 4].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 4].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 4].heading);
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 3].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 3].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 3].heading);
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 2].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 2].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 2].heading);
                        //Console.WriteLine("Line : " + iCL + " [0] " + mf.ct.ContourLineList[iCL][mpoints1 - 1].easting + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 1].northing + "  " + mf.ct.ContourLineList[iCL][mpoints1 - 1].heading);

                    }
                }

                if ((gTemp[indxA].mode == (int)TrackMode.Curve) && (gTemp[indxB].mode == (int)TrackMode.Curve)) // Line of choise from list
                {
                    mf.ct.ContourLineList?.Clear();
                    return;
                    /*
                    if (lblNameCurveA.Text == lblNameCurveB.Text)  // one line and one curve
                    {
                        for (int icurpoi = 0; icurpoi < tempListpart6.Count - 1; icurpoi++)
                        {
                            // points at the connectinglines
                            ContourPointsABLineA1.easting = tempListpart6[icurpoi].easting - Math.Cos(ContourHeadingA1B1) * DistanceAxBxmax * DistanceAxBxProcend / 100 * iCL;//* mf.m2FtOrM;
                            ContourPointsABLineA1.northing = tempListpart6[icurpoi].northing - Math.Sin(ContourHeadingA1B1) * DistanceAxBxmax * DistanceAxBxProcend / 100 * iCL;// * mf.m2FtOrM;
                                                                                                                                                                                // heading of the new AB line

                            ContourPointsABLineA1.heading = tempListpart6[icurpoi].heading;
                            mf.ct.ContourLineList[iCL].Add(ContourPointsABLineA1);
                            mf.ct.ContourLineList?.Clear();
                        }
                    } */
                }

            }

            // draws the blue curve Pattern lines
            for (int iCL = 0; iCL < mf.ct.ContourLineList.Count - 1; iCL++)
            {
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue   main lines
                GL.Begin(PrimitiveType.Lines);
                for (int iPoi = 1; iPoi < mf.ct.ContourLineList[iCL].Count - 2; iPoi++)
                {
                    GL.Vertex3(mf.ct.ContourLineList[iCL][iPoi].easting, mf.ct.ContourLineList[iCL][iPoi].northing, 0);
                }
                GL.End();
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue extentions side A1B1
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(mf.ct.ContourLineList[iCL][0].easting, mf.ct.ContourLineList[iCL][0].northing, 0);
                GL.Vertex3(mf.ct.ContourLineList[iCL][1].easting, mf.ct.ContourLineList[iCL][1].northing, 0);
                GL.End();
                GL.LineWidth(1);
                GL.Color3(0.5f, 0.5f, 1.0f);   //blue  extentions side  A2B2
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 2].northing, 0);
                GL.Vertex3(mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].easting, mf.ct.ContourLineList[iCL][mf.ct.ContourLineList[iCL].Count - 1].northing, 0);
                GL.End();
            }
        }

        private void CrossingpointsA()
        {
            int starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int isStart = 0;
            int iA;
            iA = indxA;

            if ((iA != iAbefor) || (DisAbefor != nudSetDistLineA)) // only one time to check
            {
                DisAbefor = nudSetDistLineA;
                iAbefor = iA;
                LineCountCorrect = true;
                mf.ct.ctListA.Clear();

                backupListFenceA?.Clear();

                foreach (var item in mf.bnd.bndList[0].fenceLine)
                {
                    backupListFenceA.Add(item);
                }

                //AB Lines
                if ((gTemp[iA].mode == (int)TrackMode.AB))//&& (mf.ct.ctListA.Count < 2))
                {
                    double Beast, Bnorth, Bhead;
                    int NewABLength = 500;
                    Beast = gTemp[iA].ptA.easting - (Math.Sin(gTemp[iA].heading) * NewABLength) + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    Bnorth = gTemp[iA].ptA.northing - (Math.Cos(gTemp[iA].heading) * NewABLength) - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                    Bhead = gTemp[iA].heading;
                    vec3 PointsABLine = new vec3(Beast, Bnorth, Bhead);
                    mf.ct.ctListA.Add(PointsABLine);
                    for (int iPo = 1; iPo < 150; iPo++)
                    {
                        PointsABLine.easting = Beast + (Math.Sin(gTemp[iA].heading) * 25 * iPo); // + Math.Cos(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        PointsABLine.northing = Bnorth + (Math.Cos(gTemp[iA].heading) * 25 * iPo); // - Math.Sin(gTemp[iA].heading) * nudSetDistLineA;// * mf.m2FtOrM;
                        PointsABLine.heading = gTemp[iA].heading;
                        mf.ct.ctListA.Add(PointsABLine);
                    }

                }
                else if (gTemp[iA].mode == (int)TrackMode.Curve)
                {
                    vec3 pntc = new vec3(gTemp[iA].curvePts[0]);

                    for (int ictA = 0; ictA < gTemp[iA].curvePts.Count; ictA++)
                    {
                        pntc.easting = gTemp[iA].curvePts[ictA].easting - Math.Cos(ContourHeadingA1B1) * nudSetDistLineA;// * mf.m2FtOrM;
                        pntc.northing = gTemp[iA].curvePts[ictA].northing - Math.Sin(ContourHeadingA1B1) * nudSetDistLineA;// * mf.m2FtOrM;
                        pntc.heading = gTemp[iA].curvePts[ictA].heading;
                        mf.ct.ctListA.Add(pntc);  // save the moved ABcurve
                    }
                }

                CrossingpointEast1 = 0;
                CrossingpointNorth1 = 0;
                CrossingpointEast2 = 0;
                CrossingpointNorth2 = 0;

                for (int i = 0; i < mf.ct.ctListA.Count - 1; i++)
                {
                    for (int k = 0; k < backupListFenceA.Count - 1; k++)
                    {
                        int res = GetLineIntersectionCont(
                        mf.ct.ctListA[i].easting,
                        mf.ct.ctListA[i].northing,
                        mf.ct.ctListA[i + 1].easting,
                        mf.ct.ctListA[i + 1].northing,

                        backupListFenceA[k].easting,
                        backupListFenceA[k].northing,
                        backupListFenceA[k + 1].easting,
                        backupListFenceA[k + 1].northing,
                        ref iE, ref iN);
                        if (res == 1)
                        {
                            if (isStart == 0 && CrossingpointEast1 == 0 && CrossingpointNorth1 == 0)
                            {
                                starttrack = i;
                                startbndl = k + 1;
                                // insert crossing startpoint in cutline
                                CrossPointA1.easting = CrossingpointEast;
                                CrossPointA1.northing = CrossingpointNorth;
                                CrossPointA1.heading = mf.ct.ctListA[i].heading;
                                mf.ct.ctListA.Insert(starttrack + 1, CrossPointA1);
                                // insert crossing startpoint in fenceline_new
                                vec3 pointcrossfencestart = new vec3(CrossingpointEast, CrossingpointNorth, backupListFenceA[k].heading);
                                backupListFenceA.Insert(startbndl, pointcrossfencestart);

                                CrossingpointEast1 = CrossingpointEast;
                                CrossingpointNorth1 = CrossingpointNorth;
                                CrossingpointEast = 0;
                                CrossingpointNorth = 0;

                                //isStart++;
                            }
                            else if (CrossingpointEast1 != CrossingpointEast || CrossingpointNorth1 != CrossingpointNorth)
                            {
                                endtrack = i;
                                endbndl = k + 1;
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
                CrossPointA2.easting = CrossingpointEast2;
                CrossPointA2.northing = CrossingpointNorth2;
                if ((isStart < 2) && (mf.ct.ctListA.Count == 0))
                {
                    mf.TimedMessageBox(2000, "Error", "Crossings not Found");
                    return;
                }

                CrossPointA2.heading = mf.ct.ctListA[endtrack].heading;
                mf.ct.ctListA.Insert(endtrack, CrossPointA2);
                backupListFenceA.Insert(endbndl, CrossPointA2);
                //Console.WriteLine("CrossPointA1 : " + CrossPointA1.easting + "  " + CrossPointA1.northing);
                //Console.WriteLine("CrossPointA2 : " + CrossPointA2.easting + "  " + CrossPointA2.northing);

                //for (int l = 0; l < mf.ct.ctListA.Count; l++)
                //Console.WriteLine("mf.ct.ctListA 0 : " + l + " " + mf.ct.ctListA[l].easting + " : " + mf.ct.ctListA[l].northing);

                //                   Console.WriteLine("fenceLine_new 0 :  " + mf.bnd.bndList[0].fenceLine_new[startbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[startbndl].northing);
                //                   Console.WriteLine("fenceLine_new 1 :  " + mf.bnd.bndList[0].fenceLine_new[endbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[endbndl].northing);



                //for (int l = 0; l < mf.ct.ctListA.Count; l++)
                //  Console.WriteLine("mf.ct.ctListA 0 : " + l + " " + mf.ct.ctListA[l].easting + " : " + mf.ct.ctListA[l].northing);

                tempListpart5.Clear();
                //   Console.WriteLine("mf.bndl.tracksArrbndl 2 : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[starttrack + 1].easting + " : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[starttrack + 1].northing);
                //   Console.WriteLine("mf.bndl.tracksArrbndl 3 : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].easting + " : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].northing);
                tempListpart5.Add(CrossPointA1);
                if (endtrack < starttrack)
                    for (int i = endtrack + 1; i < starttrack + 2; i++)
                    {
                        tempListpart5.Add(mf.ct.ctListA[i]);
                    }
                else
                    for (int i = starttrack + 1; i < endtrack + 1; i++)
                    {
                        tempListpart5.Add(mf.ct.ctListA[i]);
                    }
                tempListpart5.Add(CrossPointA2);

                //for (int l = 0; l < tempListpart5.Count; l++)
                //    Console.WriteLine("tempListpart5 : " + l + "  " + tempListpart5[l].easting + "  " + tempListpart5[l].northing + "  " + tempListpart5[l].heading);

                if (gTemp[iA].mode == (int)TrackMode.Curve)
                {

                    checkLineCurvePoints();
                }


                //CrossingpointsB();
            }           // end of ABline  mode
        }

        private void CrossingpointsB()
        {
            int starttrack = 0, endtrack = 0, startbndl = 0, endbndl = 0;
            int isStart = 0;
            int iB;
            iB = indxB;


            if ((iB != iBbefor) || (DisBbefor != nudSetDistLineB)) // only one time to check
            {
                DisBbefor = nudSetDistLineB;
                {
                    iBbefor = iB;
                    LineCountCorrect = true;
                    mf.ct.ctListB?.Clear();

                    backupListFenceB?.Clear();

                    foreach (var item in mf.bnd.bndList[0].fenceLine)
                    {
                        backupListFenceB.Add(item);
                    }

                    //AB Lines
                    if ((gTemp[iB].mode == (int)TrackMode.AB))//&& (mf.ct.ctListA.Count < 2))
                    {
                        double Beast, Bnorth, Bhead;
                        int NewABLength = 400;
                        Beast = gTemp[iB].ptA.easting - (Math.Sin(gTemp[iB].heading) * NewABLength) + Math.Cos(gTemp[iB].heading) * nudSetDistLineB;// * mf.m2FtOrM;
                        Bnorth = gTemp[iB].ptA.northing - (Math.Cos(gTemp[iB].heading) * NewABLength) - Math.Sin(gTemp[iB].heading) * nudSetDistLineB;// * mf.m2FtOrM;
                        Bhead = gTemp[iB].heading;
                        vec3 PointsABLine = new vec3(Beast, Bnorth, Bhead);
                        mf.ct.ctListB.Add(PointsABLine);
                        for (int iPo = 1; iPo < 150; iPo++)
                        {
                            PointsABLine.easting = Beast + (Math.Sin(gTemp[iB].heading) * 25 * iPo); // + Math.Cos(gTemp[iB].heading) * nudSetDistLineB;// * mf.m2FtOrM;
                            PointsABLine.northing = Bnorth + (Math.Cos(gTemp[iB].heading) * 25 * iPo);// - Math.Sin(gTemp[iB].heading) * nudSetDistLineB;// * mf.m2FtOrM;
                            PointsABLine.heading = gTemp[iB].heading;
                            mf.ct.ctListB.Add(PointsABLine);
                        }
                    }
                    else if (gTemp[iB].mode == (int)TrackMode.Curve)
                    {
                        vec3 pntc = new vec3(gTemp[iB].curvePts[0]);

                        for (int ictB = 0; ictB < gTemp[iB].curvePts.Count; ictB++)
                        {
                            pntc.easting = gTemp[iB].curvePts[ictB].easting - Math.Cos(ContourHeadingA1B1) * nudSetDistLineB;// * mf.m2FtOrM;
                            pntc.northing = gTemp[iB].curvePts[ictB].northing - Math.Sin(ContourHeadingA1B1) * nudSetDistLineB;// * mf.m2FtOrM;
                            pntc.heading = gTemp[iB].curvePts[ictB].heading;
                            mf.ct.ctListB.Add(pntc);  // save the moved ABcurve
                        }
                    }

                    CrossingpointEast1 = 0;
                    CrossingpointNorth1 = 0;
                    CrossingpointEast2 = 0;
                    CrossingpointNorth2 = 0;

                    for (int i = 0; i < mf.ct.ctListB.Count - 1; i++)
                    {
                        for (int k = 0; k < backupListFenceB.Count - 1; k++)
                        {

                            int res = GetLineIntersectionCont(
                    mf.ct.ctListB[i].easting,
                    mf.ct.ctListB[i].northing,
                    mf.ct.ctListB[i + 1].easting,
                    mf.ct.ctListB[i + 1].northing,

                    backupListFenceB[k].easting,
                    backupListFenceB[k].northing,
                    backupListFenceB[k + 1].easting,
                    backupListFenceB[k + 1].northing,
                    ref iE, ref iN);
                            if (res == 1)
                            {
                                if (isStart == 0 && CrossingpointEast1 == 0 && CrossingpointNorth1 == 0)
                                {
                                    starttrack = i;
                                    startbndl = k + 1;
                                    // insert crossing startpoint in cutline
                                    CrossPointB1.easting = CrossingpointEast;
                                    CrossPointB1.northing = CrossingpointNorth;
                                    CrossPointB1.heading = mf.ct.ctListB[i].heading;
                                    mf.ct.ctListB.Insert(starttrack + 1, CrossPointB1);

                                    // insert crossing startpoint in fenceline_new
                                    vec3 pointcrossfencestart = new vec3(CrossingpointEast, CrossingpointNorth, backupListFenceB[k].heading);
                                    backupListFenceB.Insert(startbndl, pointcrossfencestart);

                                    CrossingpointEast1 = CrossingpointEast;
                                    CrossingpointNorth1 = CrossingpointNorth;
                                    CrossingpointEast = 0;
                                    CrossingpointNorth = 0;

                                    // isStart++;
                                }
                                else if (CrossingpointEast1 != CrossingpointEast || CrossingpointNorth1 != CrossingpointNorth)
                                {
                                    endtrack = i;
                                    endbndl = k + 1;  // ############### + 1
                                                      //                                   if (CrossingpointEast != CrossingpointEast1 && CrossingpointNorth != CrossingpointNorth1)
                                    CrossingpointEast2 = CrossingpointEast;
                                    CrossingpointNorth2 = CrossingpointNorth;
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
                    // insert crossing endpoint in cutline
                    CrossPointB2.easting = CrossingpointEast2;
                    CrossPointB2.northing = CrossingpointNorth2;
                    CrossPointB2.heading = mf.ct.ctListB[endtrack].heading;
                    mf.ct.ctListB.Insert(endtrack, CrossPointB2);
                    backupListFenceB.Insert(endbndl, CrossPointB2);

                    //Console.WriteLine("CrossPointB1 : " + CrossPointB1.easting + "  " + CrossPointB1.northing);
                    //Console.WriteLine("CrossPointB2 : " + CrossPointB2.easting + "  " + CrossPointB2.northing);


                    //for (int l = 0; l < mf.ct.ctListB.Count; l++)
                    //Console.WriteLine("mf.ct.ctListB 0 : " + l + " " + mf.ct.ctListB[l].easting + " : " + mf.ct.ctListB[l].northing);

                    //                   Console.WriteLine("fenceLine_new 0 :  " + mf.bnd.bndList[0].fenceLine_new[startbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[startbndl].northing);
                    //                   Console.WriteLine("fenceLine_new 1 :  " + mf.bnd.bndList[0].fenceLine_new[endbndl].easting + " : " + mf.bnd.bndList[0].fenceLine_new[endbndl].northing);



                    tempListpart6.Clear();
                    //   Console.WriteLine("mf.bndl.tracksArrbndl 2 : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[starttrack + 1].easting + " : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[starttrack + 1].northing);
                    //   Console.WriteLine("mf.bndl.tracksArrbndl 3 : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].easting + " : " + mf.bndl.tracksArrbndl[mf.bndl.idxbndl].trackPtsbndl[endtrack].northing);

                    //tempListpart6.Add(backupListFenceB[startbndl]);
                    if (endtrack < starttrack)
                        for (int i = endtrack + 1; i < starttrack + 2; i++)
                        {
                            tempListpart6.Add(mf.ct.ctListB[i]);
                        }
                    else
                        for (int i = starttrack + 1; i < endtrack + 1; i++)
                        {
                            tempListpart6.Add(mf.ct.ctListB[i]);
                        }



                    //tempListpart5.Add(CrossPointB1);
                    //for (int i = starttrack; i < endtrack; i++)
                    //{
                    //   tempListpart6.Add(mf.ct.ctListB[i]);
                    //}
                    //tempListpart5.Add(CrossPointB2);

                    //for (int l = 0; l < tempListpart6.Count; l++)
                    //Console.WriteLine("tempListpart6 : " + l + " " + tempListpart6[l].easting + " : " + tempListpart6[l].northing);
                    if (gTemp[iB].mode == (int)TrackMode.Curve)
                        checkLineCurvePoints();
                    //CrossingpointsA();
                }           // end of ABline  mode,
            }
        }

        private void btnCycleBackwardFirstAB_Click(object sender, EventArgs e)
        {
            skipagain1:
            nudSetDistanceLineA.Value = 0;
            nudSetDistLineA = 0;
            if (gTemp.Count > 0)
            {
                indx = indxA;
                if (indx > 0) indx--;
                else indx = gTemp.Count - 1;
                if ((indx == indxB) && (nudSetDistanceLineA.Text == nudSetDistanceLineB.Text))
                {
                    nudSetDistanceLineA.Value = 30;
                    nudSetDistanceLineA.Text = "" + ((double)nudSetDistanceLineA.Value * mf.m2FtOrM).ToString("N2") + "";
                    nudSetDistLineA = 30;
                }
            }
            indxA = indx;
            if ((gTemp[indx].name.Length > 3 && gTemp[indx].name.Substring(0, 1) == "&"))
                goto skipagain1;
            LineCountCorrect = true;
            FixLabelsCurveA();
            CrossingpointsA();
            DrawBuiltLinesA();
            CheckCrossingpoints();
        }

        private void btnCycleForwardFirstAB_Click(object sender, EventArgs e)
        {
            skipagain2:
            nudSetDistanceLineA.Value = 0;
            nudSetDistLineA = 0;
            if (gTemp.Count > 0)
            {
                indx = indxA;
                if (indx < (gTemp.Count - 1)) indx++;
                else indx = 0;
                if ((indx == indxB) && (nudSetDistanceLineA.Text == nudSetDistanceLineB.Text))
                {
                    nudSetDistanceLineA.Value = 30;
                    nudSetDistanceLineA.Text = "" + ((double)nudSetDistanceLineA.Value * mf.m2FtOrM).ToString("N2") + "";
                    nudSetDistLineA = 30;
                }
            }
            indxA = indx;
            if ((gTemp[indx].name.Length > 3 && gTemp[indx].name.Substring(0, 1) == "&"))
                goto skipagain2;
            LineCountCorrect = true;
            FixLabelsCurveA();
            CrossingpointsA();
            DrawBuiltLinesA();
            CheckCrossingpoints();
        }

        private void btnCycleBackwardSecondAB_Click(object sender, EventArgs e)
        {
            skipagain3:
            nudSetDistanceLineB.Value = 0;
            nudSetDistLineB = 0;
            if (gTemp.Count > 0)
            {
                indx = indxB;
                if (indx > 0) indx--;
                else indx = gTemp.Count - 1;
                if ((indx == indxA) && (nudSetDistanceLineA.Text == nudSetDistanceLineB.Text))
                {
                    nudSetDistanceLineB.Value = 30;
                    nudSetDistanceLineB.Text = "" + ((double)nudSetDistanceLineB.Value * mf.m2FtOrM).ToString("N2") + "";
                    nudSetDistLineB = 30;
                }
            }
            indxB = indx;
            if ((gTemp[indx].name.Length > 3 && gTemp[indx].name.Substring(0, 1) == "&"))
                goto skipagain3;
            LineCountCorrect = true;
            FixLabelsCurveB();
            //checkLineCurvePoints();
            CrossingpointsB();
            DrawBuiltLinesB();
            CheckCrossingpoints();
        }

        private void btnCycleForwardSecondAB_Click(object sender, EventArgs e)
        {
            skipagain4:
            nudSetDistanceLineB.Value = 0;
            nudSetDistLineB = 0;
            if (gTemp.Count > 0)
            {
                indx = indxB;
                if (indx < (gTemp.Count - 1)) indx++;
                else indx = 0;
                if ((indx == indxA) && (nudSetDistanceLineA.Text == nudSetDistanceLineB.Text))
                {
                    nudSetDistanceLineB.Value = 30;
                    nudSetDistanceLineB.Text = "" + ((double)nudSetDistanceLineB.Value * mf.m2FtOrM).ToString("N2") + "";
                    nudSetDistLineB = 30;
                }
            }

            indxB = indx;
            if ((gTemp[indx].name.Length > 3 && gTemp[indx].name.Substring(0, 1) == "&"))
                goto skipagain4;
            LineCountCorrect = true;
            FixLabelsCurveB();
            CrossingpointsB();
            DrawBuiltLinesB();
            CheckCrossingpoints();
        }

        private void btnDeletePattern_Click(object sender, EventArgs e)
        {
            Properties.Settings.Default.set_CounturABLineA = indxA;
            Properties.Settings.Default.set_CounturABLineB = indxB;
            mf.FileDeletePatternTracks();
            mf.isContourPattern = false;
            mf.btnBuildTracks_small.Image = Properties.Resources.Splitlines;
            Close();
        }

        private void btnleavenosave_Click(object sender, EventArgs e)
        {
            Properties.Settings.Default.set_CounturABLineA = indxA;
            Properties.Settings.Default.set_CounturABLineB = indxB;
            mf.isContourPattern = false;
            mf.btnBuildTracks_small.Image = Properties.Resources.Splitlines;
            Close();
        }

        private void btnLessLines_Click(object sender, EventArgs e)
        {
            if (HowManyLines > 2)
                HowManyLines -= 1;
            lblCountLinesAB.Text = (HowManyLines + 1).ToString() + " Lines";
            lblNewWorkWidth.Text = ((ChooseDistanceSun / HowManyLines) * mf.m2FtOrM).ToString("N2") + mf.unitsFtM;
            if (Case == 1)
            {
                ToolwidthsunB = DistanceB2A2 / HowManyLines;
                ToolwidthsunA = DistanceA1B1 / HowManyLines;
            }
            else
            {
                if ((Case == 2) || (Case == 3))
                {
                    ToolwidthsunA = DistanceAxBxmin / HowManyLines;
                    ToolwidthsunB = DistanceAxBxmax / HowManyLines;
                    if (ToolwidthsunA > ToolwidthsunB)
                        (ToolwidthsunA, ToolwidthsunB) = (ToolwidthsunB, ToolwidthsunA);
                }
            }
            LineCountCorrect = false;
            CheckCrossingpoints();
            LineCountCorrect = false;
        }

        private void btnMoreLines_Click(object sender, EventArgs e)
        {

            HowManyLines += 1;
            lblCountLinesAB.Text = (HowManyLines + 1).ToString() + " Lines";
            lblNewWorkWidth.Text = ((ChooseDistanceSun / HowManyLines) * mf.m2FtOrM).ToString("N2") + mf.unitsFtM;
            if (Case == 1)
            {
                ToolwidthsunB = DistanceB2A2 / HowManyLines;
                ToolwidthsunA = DistanceA1B1 / HowManyLines;
            }
            else
            {
                if ((Case == 2) || (Case == 3))
                {
                    ToolwidthsunA = DistanceAxBxmin / HowManyLines;
                    ToolwidthsunB = DistanceAxBxmax / HowManyLines;
                    if (ToolwidthsunA > ToolwidthsunB)
                        (ToolwidthsunA, ToolwidthsunB) = (ToolwidthsunB, ToolwidthsunA);
                }
            }
            LineCountCorrect = false;
            CheckCrossingpoints();
            LineCountCorrect = false;
        }

        private void btnZoomPlus_Click(object sender, EventArgs e)
        {
            zoom -= 0.1;
            if (zoom < 0.1) zoom = 0.1;
        }

        private void btnZoomMinus_Click(object sender, EventArgs e)
        {
            zoom += 0.1;
            if (zoom > 1.5) zoom = 1.5;
        }

        private void btnExtentMinus_Click(object sender, EventArgs e)
        {
            if (Extentions > MinExtentions) Extentions -= 5;
            Properties.Settings.Default.set_Extentions = Extentions;
        }

        private void btnExtentPlus_Click(object sender, EventArgs e)
        {
            if (Extentions < 200) Extentions += 5;
            Properties.Settings.Default.set_Extentions = Extentions;
        }

        private void nudSetDistanceLineA_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            nudSetDistLineA = (double)nudSetDistanceLineA.Value;
            DrawBuiltLinesA();
            iAbefor = -1;
        }

        private void nudSetDistanceLineB_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            nudSetDistLineB = (double)nudSetDistanceLineB.Value;
            DrawBuiltLinesB();
            iBbefor = -1;
        }

        private void oglSelfCont_Resize(object sender, EventArgs e)
        {
            oglSelfCont.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            GL.Viewport(0, 0, oglSelfCont.Width, oglSelfCont.Height);

            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);
        }

        bool crossingpoint = true;

        public int GetLineIntersectionCont(double p0x, double p0y, double p1x, double p1y,
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

        private void FixLabelsCurveA()
        {
            if (indxA > -1 && gTemp.Count > 0)
            {
                if (mf.trk.gArr.Count > 2)
                    lblNameCurveA.Text = indxA + " " + gTemp[indxA].name;
                else
                    lblNameCurveA.Text = gTemp[0].name;
                lblNameCurveA.Enabled = true;
            }
            else
            {
                lblNameCurveB.Text = "***";
                lblNameCurveB.Enabled = false;
            }
            CheckLineCurve();
        }

        private void CheckLineCurve()
        {
            if ((gTemp[indxB].mode == (int)TrackMode.Curve) && (gTemp[indxA].mode == (int)TrackMode.AB))
                (indxA, indxB) = (indxB, indxA);

            if ((gTemp[indxA].mode == 4) && (gTemp[indxB].mode == 4))
                Case = 4;
            else
            {
                if ((gTemp[indxA].mode == 2) && (gTemp[indxB].mode == 2))
                {
                    Case = 1;
                }
                else if ((gTemp[indxA].mode == 2) && (gTemp[indxB].mode == 4)) Case = 2;
                else if ((gTemp[indxA].mode == 4) && (gTemp[indxB].mode == 2)) Case = 3;
            }
        }

        private void FixLabelsCurveB()
        {
            if (indxB > -1 && gTemp.Count > 0)
            {
                if (mf.trk.gArr.Count > 2)
                    lblNameCurveB.Text = indxB + " " + gTemp[indxB].name;
                else
                    lblNameCurveA.Text = gTemp[1].name;
                lblNameCurveB.Enabled = true;
            }
            else
            {
                lblNameCurveA.Text = "***";
                lblNameCurveA.Enabled = false;
            }
            CheckLineCurve();
        }

        private void oglSelfCont_Load(object sender, EventArgs e)
        {
            oglSelfCont.MakeCurrent();
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.ClearColor(0.22f, 0.22f, 0.22f, 1.0f);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
        }

        //for calculating for display the averaged new line
        private void tlp1Contour_Paint(object sender, PaintEventArgs e)
        {

        }
    }
}
