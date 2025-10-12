using System;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormFlags : Form
    {
        //class variables
        private readonly FormGPS mf = null;

        private int nextflag;


        public FormFlags(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;
            InitializeComponent();
            UpdateLabels();
        }

        private void UpdateLabels()
        {
            lblLatStart.Text = mf.flagPts[mf.flagNumberPicked - 1].latitude.ToString();
            lblLonStart.Text = mf.flagPts[mf.flagNumberPicked - 1].longitude.ToString();
            lblEasting.Text = mf.flagPts[mf.flagNumberPicked - 1].easting.ToString("N2");
            lblNorthing.Text = mf.flagPts[mf.flagNumberPicked - 1].northing.ToString("N2");
            lblHeading.Text = glm.toDegrees(mf.flagPts[mf.flagNumberPicked - 1].heading).ToString("N2");
            lblFlagSelected.Text = mf.flagPts[mf.flagNumberPicked - 1].ID.ToString();
            tboxFlagNotes.Text = mf.flagPts[mf.flagNumberPicked - 1].notes;
            if (checkFlagAlarm.Checked)
                mf.FlagAlarm = true;
            else
                mf.FlagAlarm = false;
            //checkFlagAlarm.Checked = false;
        }

        private void FormFlags_Load(object sender, EventArgs e)
        {
            UpdateLabels();

            if (!mf.IsOnScreen(Location, Size, 1))
            {
                Top = 0;
                Left = 0;
            }
        }

        private void btnNorth_MouseDown(object sender, MouseEventArgs e)
        {
            //if (mf.flagNumberPicked < mf.flagPts.Count)
            mf.flagNumberPicked++;
            if (mf.flagNumberPicked > mf.flagPts.Count) mf.flagNumberPicked = 1;
            UpdateLabels();
        }

        private void btnSouth_MouseDown(object sender, MouseEventArgs e)
        {
            //if (mf.flagNumberPicked > 1)
            mf.flagNumberPicked--;
            if (mf.flagNumberPicked < 1) mf.flagNumberPicked = mf.flagPts.Count;
            UpdateLabels();
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            mf.flagNumberPicked = 0;
            mf.FileSaveFlags();
            Close();
        }

        private void btnDeleteFlag_Click(object sender, EventArgs e)
        {
            int flag = mf.flagNumberPicked;
            if (mf.flagPts.Count > 0) mf.DeleteSelectedFlag();
            if (mf.flagPts.Count == 0)
            {
                mf.FileSaveFlags();
                mf.isFlagMenuOpen = false;
                Close();
                return;
            }
            if (flag > mf.flagPts.Count) mf.flagNumberPicked = mf.flagPts.Count;
            else mf.flagNumberPicked = flag;
            UpdateLabels();
        }

        private void tboxFlagNotes_Leave(object sender, EventArgs e)
        {
            if (mf.flagNumberPicked > 0)
                mf.flagPts[mf.flagNumberPicked - 1].notes = tboxFlagNotes.Text;
        }

        private void tboxFlagNotes_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == '\r') e.Handled = true;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //MakeDubinsLineFromPivotToFlag();
            if (mf.flagPts.Count > 0)
            {
                if (mf.flagNumberPicked == 0) mf.flagNumberPicked = mf.flagPts.Count;

                if (mf.flagNumberPicked > mf.flagPts.Count) mf.flagNumberPicked = mf.flagPts.Count;

                if (mf.isMetric)
                    lblDistanceToFlag.Text = glm.Distance(mf.pn.fix,
                        mf.flagPts[mf.flagNumberPicked - 1].easting, mf.flagPts[mf.flagNumberPicked - 1].northing).ToString("N2") + " m";
                else lblDistanceToFlag.Text = (glm.Distance(mf.pn.fix,
                    mf.flagPts[mf.flagNumberPicked - 1].easting, mf.flagPts[mf.flagNumberPicked - 1].northing) * glm.m2ft).ToString("N2") + " m";
                
                //UpdateLabels();
            }
        }

        private void tboxFlagNotes_Click(object sender, EventArgs e)
        {
            if (mf.isKeyboardOn)
            {
                mf.KeyboardToText((TextBox)sender, this);
                btnExit.Focus();
            }
        }

        private void checkFlagAlarm_CheckedChanged(object sender, EventArgs e)
        {
            if (checkFlagAlarm.Checked)
                mf.FlagAlarm = true;
            else
                mf.FlagAlarm = false;
            UpdateLabels();
        }

        public int soundflag;
        private void btnRed_Click(object sender, EventArgs e)
        {
            Button btnRed = (Button)sender;

            if (btnRed.Name == "btnRed")
            {
                mf.flagColor = 0;
            }
            else if (btnRed.Name == "btnGreen")
            {
                mf.flagColor = 1;
            }
            else if (btnRed.Name == "btnYellow")
            {
                mf.flagColor = 2;
            }

            nextflag = mf.flagPts.Count + 1;
            //Console.Write("nextflag "); Console.WriteLine(nextflag);
 
            FlagPosition();
        }

        private void FlagPosition()
        {
   
            double lati, longi, northi, easti;
            double Distance_Flag_pivot;

            Distance_Flag_pivot = (double)nudSetDistanceFlag.Value;

            northi = mf.pivotAxlePos.northing + (Math.Cos(mf.fixHeading) * Distance_Flag_pivot);
            easti = mf.pivotAxlePos.easting + (Math.Sin(mf.fixHeading) * Distance_Flag_pivot);
            mf.pn.ConvertLocalToWGS84(northi, easti, out lati, out longi);
            Console.WriteLine("CS   ");
            Console.WriteLine("CS northi   " + northi);
            Console.WriteLine("CS easti    " + easti);
            Console.WriteLine("CS pn.lati   " + lati);
            Console.WriteLine("CS pn.longi  " + longi);
            if (mf.FlagAlarm)
                soundflag = 1;
            else
                soundflag = 0;
            CFlag flagPt = new CFlag(lati, longi, easti, northi,
                mf.fixHeading, mf.flagColor, nextflag, soundflag, nextflag.ToString());

            mf.flagPts.Add(flagPt);
            mf.isFlagMenuOpen = true;
            mf.FileSaveFlags();
            mf.flagNumberPicked++;
            UpdateLabels();
        }

        private void nudSetDistanceFlag_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NudlessNumericUpDown)sender, this);
            btnExit.Focus();
        }

        private void label7_Click(object sender, EventArgs e)
        {

        }

        private void btnYellow_Click(object sender, EventArgs e)
        {

        }

        private void btnGreen_Click(object sender, EventArgs e)
        {

        }

    }
}