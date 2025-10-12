using System;
using System.Drawing;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormYouTurn : Form
    {
        private readonly FormGPS mf = null;

        //public bool isYouTurnRightA, isYouTurnRightB;
        //public int rowSkipsWidth2 = 1, rowSkipsWidth1 = 1;
        public bool isHeadingAB = true;

        bool right_leftA = true, right_leftB = true;

        public int idxyt = 0;



        public FormYouTurn(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;
            FormYouTurnAch Ach = new FormYouTurnAch(mf);

            InitializeComponent();
            mf.FileLoadYouturnLines();
            Ach.BNTbndLoopyt();
            UpDateYouturnForm();

            mf.yt.isYouTurnBtnOn = false;
            mf.btnAutoYouTurn.Image = Properties.Resources.YouTurnNo;

        }

        public void UpDateYouturnForm()
        {
            cboxpRowWidthA.Text = Convert.ToString(Properties.Settings.Default.set_youSkipWidth);
            cboxpRowWidthB.Text = Convert.ToString(Properties.Settings.Default.set_youSkipWidth1);
            mf.yt.isYouTurnRightA = Properties.Settings.Default.btnYouTurnA;
            mf.yt.isYouTurnRightB = Properties.Settings.Default.btnYouTurnB;
            mf.yt.isYouTurncircleA = Properties.Settings.Default.set_isYouTurncircleRightA;
            mf.yt.isYouTurncircleB = Properties.Settings.Default.set_isYouTurncircleRightB;

            if (mf.yt.isABDirectionYouturn)
            {
                mf.yt.isYouTurnRight = mf.yt.isYouTurnRightA;
                mf.yt.rowSkipsWidth = mf.yt.rowSkipsWidth1;
            }
            else
            {
                mf.yt.isYouTurnRight = mf.yt.isYouTurnRightB;
                mf.yt.rowSkipsWidth = mf.yt.rowSkipsWidth2;
            }
            if (!mf.yt.isYouTurnRightA) { btnYouTurnA.Text = "A right"; btnYouTurnA.Image = AgOpenGPS.Properties.Resources.ArrowRight; }
            else { btnYouTurnA.Text = "A left"; btnYouTurnA.Image = AgOpenGPS.Properties.Resources.ArrowLeft; }
            if (!mf.yt.isYouTurnRightB) { btnYouTurnB.Text = "B right"; btnYouTurnB.Image = AgOpenGPS.Properties.Resources.ArrowRight; }
            else { btnYouTurnB.Text = "B left"; btnYouTurnB.Image = AgOpenGPS.Properties.Resources.ArrowLeft; }

            if (!Properties.Settings.Default.set_turncircleA)
            {
                btnturncircleA.Image = AgOpenGPS.Properties.Resources.TurnCurve_off;
            }
            else
            {
                btnturncircleA.Image = AgOpenGPS.Properties.Resources.TurnCurve_B;
            }
            if (!Properties.Settings.Default.set_turncircleB)
            {
                btnturncircleB.Image = AgOpenGPS.Properties.Resources.TurnCurve_off;
            }
            else
            {
                btnturncircleB.Image = AgOpenGPS.Properties.Resources.TurnCurve_B;
            }
            mf.UpdateFixPosition();
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            Close();
        }

        private void btnYouTurnA_Click(object sender, EventArgs e)
        {
            if (!right_leftA)
            {
                btnYouTurnA.Text = "A right";
                btnYouTurnA.Image = AgOpenGPS.Properties.Resources.ArrowRight;
                mf.yt.isYouTurnRightA = false;
            }
            else
            {
                btnYouTurnA.Text = "A left";
                btnYouTurnA.Image = AgOpenGPS.Properties.Resources.ArrowLeft;
                mf.yt.isYouTurnRightA = true;
            }
            right_leftA = !right_leftA;
            Properties.Settings.Default.btnYouTurnA = mf.yt.isYouTurnRightA;
            Properties.Settings.Default.Save();
            UpDateYouturnForm();
        }

        private void btnYouTurnB_Click(object sender, EventArgs e)
        {
            if (!right_leftB)
            {
                btnYouTurnB.Text = "B right";
                btnYouTurnB.Image = AgOpenGPS.Properties.Resources.ArrowRight;
                mf.yt.isYouTurnRightB = false;
            }
            else
            {
                btnYouTurnB.Text = "B left";
                btnYouTurnB.Image = AgOpenGPS.Properties.Resources.ArrowLeft;
                mf.yt.isYouTurnRightB = true;
            }
            right_leftB = !right_leftB;
            Properties.Settings.Default.btnYouTurnB = mf.yt.isYouTurnRightB;
            Properties.Settings.Default.Save();
            UpDateYouturnForm();
        }

        private void cboxpRowWidthA_SelectedIndexChanged(object sender, EventArgs e)
        {
            mf.yt.rowSkipsWidth1 = cboxpRowWidthA.SelectedIndex + 1;// +1
            if (!mf.yt.isYouTurnTriggered) mf.yt.ResetCreatedYouTurn();
            Properties.Settings.Default.set_youSkipWidth = mf.yt.rowSkipsWidth1 - 1;
            Properties.Settings.Default.Save();
            YouTurnABskipdirection();
            mf.DrawManUTurnBtn();
        }

        private void cboxpRowWidthB_SelectedIndexChanged(object sender, EventArgs e)
        {
            mf.yt.rowSkipsWidth2 = cboxpRowWidthB.SelectedIndex + 1;// +1
            if (!mf.yt.isYouTurnTriggered) mf.yt.ResetCreatedYouTurn();
            Properties.Settings.Default.set_youSkipWidth1 = mf.yt.rowSkipsWidth2 - 1;
            Properties.Settings.Default.Save();
            YouTurnABskipdirection();
            mf.DrawManUTurnBtn();
        }

        private void timer1_Tick_1(object sender, EventArgs e)
        {

        }

        public void btnFormYouTurnAch_Click(object sender, EventArgs e)
        {
            UpDateYouturnForm();
            Close();
            mf.Fenster5();
        }

        public void YouTurnABskipdirection()  //################################
        {
            if (!mf.yt.isABDirectionYouturn)
            {
                btnYouTurnA.BackColor = Color.Red;
                btnYouTurnB.BackColor = Color.Lime;
                cboxpRowWidthA.BackColor = Color.Red;
                cboxpRowWidthB.BackColor = Color.Lime;
            }
            else
            {
                cboxpRowWidthA.BackColor = Color.Lime;
                cboxpRowWidthB.BackColor = Color.Red;
            }
            if (mf.yt.rowSkipsWidth2 < 1)
            {
                btnYouTurnB.BackColor = Color.Gray;
            }
            if (mf.yt.rowSkipsWidth1 < 1)
            {
                btnYouTurnA.BackColor = Color.Gray;
            }
        }

        private void btnturncircleA_Click(object sender, EventArgs e)
        {
            if (!mf.yt.isYouTurncircleA)
            {

                btnturncircleA.Image = AgOpenGPS.Properties.Resources.TurnCurve_B;
                Properties.Settings.Default.set_turncircleA = true;
                mf.yt.isYouTurncircleA = true;
            }
            else
            {
                btnturncircleA.Image = AgOpenGPS.Properties.Resources.TurnCurve_off;
                Properties.Settings.Default.set_turncircleA = false;
                mf.yt.isYouTurncircleA = false;
            }
            Properties.Settings.Default.set_isYouTurncircleRightA = mf.yt.isYouTurncircleA;
            UpDateYouturnForm();
        }

        private void btnturncircleB_Click(object sender, EventArgs e)
        {
            if (!mf.yt.isYouTurncircleB)
            {
                btnturncircleB.Image = AgOpenGPS.Properties.Resources.TurnCurve_B;
                Properties.Settings.Default.set_turncircleB = true;
                mf.yt.isYouTurncircleB = true;
            }
            else
            {
                btnturncircleB.Image = AgOpenGPS.Properties.Resources.TurnCurve_off;
                Properties.Settings.Default.set_turncircleB = false;
                mf.yt.isYouTurncircleB = false;
            }
            Properties.Settings.Default.set_isYouTurncircleRightB = mf.yt.isYouTurncircleB;
            UpDateYouturnForm();
        }

    }
}
