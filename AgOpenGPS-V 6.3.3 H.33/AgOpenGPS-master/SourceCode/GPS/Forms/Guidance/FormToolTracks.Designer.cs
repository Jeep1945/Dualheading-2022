
namespace AgOpenGPS.Forms.Guidance
{
    partial class FormToolTracks
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.btnToolTrackDelete = new System.Windows.Forms.Button();
            this.btnExit = new System.Windows.Forms.Button();
            this.btnAddToolTrackPts = new System.Windows.Forms.Button();
            this.btnToolAtWork = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.btnleaveMenue = new System.Windows.Forms.Button();
            this.label4 = new System.Windows.Forms.Label();
            this.btnSetupopenclose = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.hsbarToolLookAhead = new System.Windows.Forms.HScrollBar();
            this.hToolPWM = new System.Windows.Forms.HScrollBar();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.lblToolLookAhead = new System.Windows.Forms.Label();
            this.lblToolPWM = new System.Windows.Forms.Label();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.rbtnToolAntennaCenter = new System.Windows.Forms.RadioButton();
            this.rbtnToolAntennaRight = new System.Windows.Forms.RadioButton();
            this.rbtnToolAntennaLeft = new System.Windows.Forms.RadioButton();
            this.nudSetToolOffset = new AgOpenGPS.NudlessNumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.nudToolbehindPivot = new AgOpenGPS.NudlessNumericUpDown();
            this.btnSlideRoll = new System.Windows.Forms.Button();
            this.btnThirdAntenna = new System.Windows.Forms.Button();
            this.btnSlopeDirection = new System.Windows.Forms.Button();
            this.btnAddnewSlope = new System.Windows.Forms.Button();
            this.nudSetToolAntHight1 = new AgOpenGPS.NudlessNumericUpDown();
            this.groupBox5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetToolOffset)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudToolbehindPivot)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetToolAntHight1)).BeginInit();
            this.SuspendLayout();
            // 
            // btnToolTrackDelete
            // 
            this.btnToolTrackDelete.FlatAppearance.BorderColor = System.Drawing.Color.Blue;
            this.btnToolTrackDelete.Image = global::AgOpenGPS.Properties.Resources.ABTrackCurveDelete;
            this.btnToolTrackDelete.Location = new System.Drawing.Point(10, 224);
            this.btnToolTrackDelete.Name = "btnToolTrackDelete";
            this.btnToolTrackDelete.Size = new System.Drawing.Size(70, 70);
            this.btnToolTrackDelete.TabIndex = 3;
            this.btnToolTrackDelete.UseVisualStyleBackColor = false;
            this.btnToolTrackDelete.Click += new System.EventHandler(this.btnToolTrackDelete_Click);
            // 
            // btnExit
            // 
            this.btnExit.Image = global::AgOpenGPS.Properties.Resources.FileSave;
            this.btnExit.Location = new System.Drawing.Point(12, 299);
            this.btnExit.Name = "btnExit";
            this.btnExit.Size = new System.Drawing.Size(70, 70);
            this.btnExit.TabIndex = 2;
            this.btnExit.UseVisualStyleBackColor = true;
            this.btnExit.Click += new System.EventHandler(this.btnExit_Click);
            // 
            // btnAddToolTrackPts
            // 
            this.btnAddToolTrackPts.Image = global::AgOpenGPS.Properties.Resources.BoundaryRecord;
            this.btnAddToolTrackPts.Location = new System.Drawing.Point(11, 76);
            this.btnAddToolTrackPts.Name = "btnAddToolTrackPts";
            this.btnAddToolTrackPts.Size = new System.Drawing.Size(70, 70);
            this.btnAddToolTrackPts.TabIndex = 0;
            this.btnAddToolTrackPts.UseVisualStyleBackColor = true;
            this.btnAddToolTrackPts.Click += new System.EventHandler(this.btnAddToolTrackPts_Click);
            // 
            // btnToolAtWork
            // 
            this.btnToolAtWork.FlatAppearance.BorderColor = System.Drawing.Color.Blue;
            this.btnToolAtWork.Image = global::AgOpenGPS.Properties.Resources.AutoSteerOff;
            this.btnToolAtWork.Location = new System.Drawing.Point(11, 369);
            this.btnToolAtWork.Name = "btnToolAtWork";
            this.btnToolAtWork.Size = new System.Drawing.Size(70, 70);
            this.btnToolAtWork.TabIndex = 4;
            this.btnToolAtWork.UseVisualStyleBackColor = false;
            this.btnToolAtWork.Click += new System.EventHandler(this.btnToolAtWork_Click);
            // 
            // label1
            // 
            this.label1.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Black;
            this.label1.Location = new System.Drawing.Point(130, 184);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(179, 48);
            this.label1.TabIndex = 573;
            this.label1.Text = "Antenna height cm";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // label2
            // 
            this.label2.Font = new System.Drawing.Font("Tahoma", 18F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Black;
            this.label2.Location = new System.Drawing.Point(87, -1);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(313, 37);
            this.label2.TabIndex = 574;
            this.label2.Text = "Track Antenna Setup";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label2.Click += new System.EventHandler(this.label2_Click);
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // btnleaveMenue
            // 
            this.btnleaveMenue.BackColor = System.Drawing.SystemColors.InactiveCaption;
            this.btnleaveMenue.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnleaveMenue.FlatAppearance.BorderSize = 0;
            this.btnleaveMenue.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnleaveMenue.Image = global::AgOpenGPS.Properties.Resources.SwitchOff;
            this.btnleaveMenue.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnleaveMenue.Location = new System.Drawing.Point(11, 441);
            this.btnleaveMenue.Name = "btnleaveMenue";
            this.btnleaveMenue.Size = new System.Drawing.Size(70, 70);
            this.btnleaveMenue.TabIndex = 577;
            this.btnleaveMenue.UseVisualStyleBackColor = false;
            this.btnleaveMenue.Click += new System.EventHandler(this.btnleaveMenue_Click);
            // 
            // label4
            // 
            this.label4.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.Color.Black;
            this.label4.Location = new System.Drawing.Point(134, 227);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(176, 48);
            this.label4.TabIndex = 578;
            this.label4.Text = "Antenna offset cm";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label4.Click += new System.EventHandler(this.label4_Click);
            // 
            // btnSetupopenclose
            // 
            this.btnSetupopenclose.BackColor = System.Drawing.Color.Chartreuse;
            this.btnSetupopenclose.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnSetupopenclose.FlatAppearance.BorderColor = System.Drawing.Color.DarkOrange;
            this.btnSetupopenclose.FlatAppearance.BorderSize = 0;
            this.btnSetupopenclose.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnSetupopenclose.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnSetupopenclose.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSetupopenclose.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnSetupopenclose.Image = global::AgOpenGPS.Properties.Resources.ArrowRight;
            this.btnSetupopenclose.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnSetupopenclose.Location = new System.Drawing.Point(10, 22);
            this.btnSetupopenclose.Name = "btnSetupopenclose";
            this.btnSetupopenclose.Size = new System.Drawing.Size(71, 44);
            this.btnSetupopenclose.TabIndex = 581;
            this.btnSetupopenclose.UseVisualStyleBackColor = false;
            this.btnSetupopenclose.Click += new System.EventHandler(this.btnSetupopenclose_Click);
            // 
            // label5
            // 
            this.label5.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.Color.Black;
            this.label5.Location = new System.Drawing.Point(158, 353);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(152, 32);
            this.label5.TabIndex = 585;
            this.label5.Text = "Look Ahead";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label5.Click += new System.EventHandler(this.label5_Click);
            // 
            // label6
            // 
            this.label6.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.Color.Black;
            this.label6.Location = new System.Drawing.Point(158, 429);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(129, 32);
            this.label6.TabIndex = 586;
            this.label6.Text = "PWM ";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label6.Click += new System.EventHandler(this.label6_Click);
            // 
            // hsbarToolLookAhead
            // 
            this.hsbarToolLookAhead.LargeChange = 1;
            this.hsbarToolLookAhead.Location = new System.Drawing.Point(163, 391);
            this.hsbarToolLookAhead.Maximum = 10;
            this.hsbarToolLookAhead.Name = "hsbarToolLookAhead";
            this.hsbarToolLookAhead.Size = new System.Drawing.Size(235, 33);
            this.hsbarToolLookAhead.TabIndex = 587;
            this.hsbarToolLookAhead.Value = 3;
            this.hsbarToolLookAhead.Scroll += new System.Windows.Forms.ScrollEventHandler(this.hsbarToolLookAhead_Scroll);
            this.hsbarToolLookAhead.ValueChanged += new System.EventHandler(this.hsbarToolLookAhead_ValueChanged);
            // 
            // hToolPWM
            // 
            this.hToolPWM.LargeChange = 1;
            this.hToolPWM.Location = new System.Drawing.Point(163, 461);
            this.hToolPWM.Maximum = 255;
            this.hToolPWM.Name = "hToolPWM";
            this.hToolPWM.Size = new System.Drawing.Size(235, 33);
            this.hToolPWM.TabIndex = 588;
            this.hToolPWM.Value = 30;
            this.hToolPWM.Scroll += new System.Windows.Forms.ScrollEventHandler(this.hToolPWM_Scroll);
            this.hToolPWM.ValueChanged += new System.EventHandler(this.hToolPWM_ValueChanged);
            // 
            // label7
            // 
            this.label7.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.Color.Black;
            this.label7.Location = new System.Drawing.Point(126, 391);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(34, 35);
            this.label7.TabIndex = 591;
            this.label7.Text = "0";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label7.Click += new System.EventHandler(this.label7_Click);
            // 
            // label8
            // 
            this.label8.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ForeColor = System.Drawing.Color.Black;
            this.label8.Location = new System.Drawing.Point(400, 391);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(34, 35);
            this.label8.TabIndex = 592;
            this.label8.Text = "10";
            this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label8.Click += new System.EventHandler(this.label8_Click);
            // 
            // label9
            // 
            this.label9.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.Color.Black;
            this.label9.Location = new System.Drawing.Point(126, 459);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(34, 35);
            this.label9.TabIndex = 593;
            this.label9.Text = "0";
            this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label9.Click += new System.EventHandler(this.label9_Click);
            // 
            // label10
            // 
            this.label10.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.Color.Black;
            this.label10.Location = new System.Drawing.Point(401, 459);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(48, 35);
            this.label10.TabIndex = 594;
            this.label10.Text = "255";
            this.label10.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label10.Click += new System.EventHandler(this.label10_Click);
            // 
            // lblToolLookAhead
            // 
            this.lblToolLookAhead.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblToolLookAhead.ForeColor = System.Drawing.Color.Black;
            this.lblToolLookAhead.Location = new System.Drawing.Point(335, 353);
            this.lblToolLookAhead.Name = "lblToolLookAhead";
            this.lblToolLookAhead.Size = new System.Drawing.Size(34, 35);
            this.lblToolLookAhead.TabIndex = 596;
            this.lblToolLookAhead.Text = "88";
            this.lblToolLookAhead.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.lblToolLookAhead.Click += new System.EventHandler(this.lblToolLookAhead_Click);
            // 
            // lblToolPWM
            // 
            this.lblToolPWM.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblToolPWM.ForeColor = System.Drawing.Color.Black;
            this.lblToolPWM.Location = new System.Drawing.Point(315, 426);
            this.lblToolPWM.Name = "lblToolPWM";
            this.lblToolPWM.Size = new System.Drawing.Size(54, 35);
            this.lblToolPWM.TabIndex = 597;
            this.lblToolPWM.Text = "88";
            this.lblToolPWM.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.lblToolPWM.Click += new System.EventHandler(this.lblToolPWM_Click);
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.rbtnToolAntennaCenter);
            this.groupBox5.Controls.Add(this.rbtnToolAntennaRight);
            this.groupBox5.Controls.Add(this.rbtnToolAntennaLeft);
            this.groupBox5.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox5.Location = new System.Drawing.Point(134, 274);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(279, 84);
            this.groupBox5.TabIndex = 598;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Antenna Offset";
            this.groupBox5.Enter += new System.EventHandler(this.groupBox5_Enter);
            // 
            // rbtnToolAntennaCenter
            // 
            this.rbtnToolAntennaCenter.Appearance = System.Windows.Forms.Appearance.Button;
            this.rbtnToolAntennaCenter.BackColor = System.Drawing.Color.Transparent;
            this.rbtnToolAntennaCenter.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.rbtnToolAntennaCenter.FlatAppearance.CheckedBackColor = System.Drawing.Color.LightGreen;
            this.rbtnToolAntennaCenter.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.rbtnToolAntennaCenter.Image = global::AgOpenGPS.Properties.Resources.AntennaNoOffset;
            this.rbtnToolAntennaCenter.Location = new System.Drawing.Point(118, 25);
            this.rbtnToolAntennaCenter.Name = "rbtnToolAntennaCenter";
            this.rbtnToolAntennaCenter.Size = new System.Drawing.Size(65, 50);
            this.rbtnToolAntennaCenter.TabIndex = 480;
            this.rbtnToolAntennaCenter.UseVisualStyleBackColor = false;
            this.rbtnToolAntennaCenter.CheckedChanged += new System.EventHandler(this.rbtnToolAntennaLeft_CheckedChanged);
            // 
            // rbtnToolAntennaRight
            // 
            this.rbtnToolAntennaRight.Appearance = System.Windows.Forms.Appearance.Button;
            this.rbtnToolAntennaRight.BackColor = System.Drawing.Color.Transparent;
            this.rbtnToolAntennaRight.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.rbtnToolAntennaRight.FlatAppearance.CheckedBackColor = System.Drawing.Color.LightGreen;
            this.rbtnToolAntennaRight.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.rbtnToolAntennaRight.Image = global::AgOpenGPS.Properties.Resources.AntennaRightOffset;
            this.rbtnToolAntennaRight.Location = new System.Drawing.Point(198, 25);
            this.rbtnToolAntennaRight.Name = "rbtnToolAntennaRight";
            this.rbtnToolAntennaRight.Size = new System.Drawing.Size(68, 50);
            this.rbtnToolAntennaRight.TabIndex = 479;
            this.rbtnToolAntennaRight.UseVisualStyleBackColor = false;
            this.rbtnToolAntennaRight.CheckedChanged += new System.EventHandler(this.rbtnToolAntennaLeft_CheckedChanged);
            // 
            // rbtnToolAntennaLeft
            // 
            this.rbtnToolAntennaLeft.Appearance = System.Windows.Forms.Appearance.Button;
            this.rbtnToolAntennaLeft.BackColor = System.Drawing.Color.Transparent;
            this.rbtnToolAntennaLeft.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.rbtnToolAntennaLeft.FlatAppearance.CheckedBackColor = System.Drawing.Color.LightGreen;
            this.rbtnToolAntennaLeft.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.rbtnToolAntennaLeft.Image = global::AgOpenGPS.Properties.Resources.AntennaLeftOffset;
            this.rbtnToolAntennaLeft.Location = new System.Drawing.Point(28, 25);
            this.rbtnToolAntennaLeft.Name = "rbtnToolAntennaLeft";
            this.rbtnToolAntennaLeft.Size = new System.Drawing.Size(73, 50);
            this.rbtnToolAntennaLeft.TabIndex = 478;
            this.rbtnToolAntennaLeft.UseVisualStyleBackColor = false;
            this.rbtnToolAntennaLeft.CheckedChanged += new System.EventHandler(this.rbtnToolAntennaLeft_CheckedChanged);
            // 
            // nudSetToolOffset
            // 
            this.nudSetToolOffset.BackColor = System.Drawing.Color.White;
            this.nudSetToolOffset.Font = new System.Drawing.Font("Tahoma", 16F, System.Drawing.FontStyle.Bold);
            this.nudSetToolOffset.Location = new System.Drawing.Point(315, 235);
            this.nudSetToolOffset.Maximum = new decimal(new int[] {
            500,
            0,
            0,
            0});
            this.nudSetToolOffset.Minimum = new decimal(new int[] {
            500,
            0,
            0,
            -2147483648});
            this.nudSetToolOffset.Name = "nudSetToolOffset";
            this.nudSetToolOffset.ReadOnly = true;
            this.nudSetToolOffset.Size = new System.Drawing.Size(98, 33);
            this.nudSetToolOffset.TabIndex = 579;
            this.nudSetToolOffset.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetToolOffset.ValueChanged += new System.EventHandler(this.nudSetToolOffset_ValueChanged);
            this.nudSetToolOffset.Click += new System.EventHandler(this.nudSetToolOffset_Click);
            // 
            // label3
            // 
            this.label3.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.Color.Black;
            this.label3.Location = new System.Drawing.Point(130, 140);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(179, 48);
            this.label3.TabIndex = 599;
            this.label3.Text = "Antenna behind Tractor cm";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.label3.Click += new System.EventHandler(this.label3_Click);
            // 
            // nudToolbehindPivot
            // 
            this.nudToolbehindPivot.BackColor = System.Drawing.Color.White;
            this.nudToolbehindPivot.Font = new System.Drawing.Font("Tahoma", 16F, System.Drawing.FontStyle.Bold);
            this.nudToolbehindPivot.Location = new System.Drawing.Point(315, 148);
            this.nudToolbehindPivot.Maximum = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            this.nudToolbehindPivot.Name = "nudToolbehindPivot";
            this.nudToolbehindPivot.ReadOnly = true;
            this.nudToolbehindPivot.Size = new System.Drawing.Size(98, 33);
            this.nudToolbehindPivot.TabIndex = 600;
            this.nudToolbehindPivot.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudToolbehindPivot.ValueChanged += new System.EventHandler(this.nudToolbehindPivot_ValueChanged);
            this.nudToolbehindPivot.Click += new System.EventHandler(this.nudToolbehindPivot_Click);
            // 
            // btnSlideRoll
            // 
            this.btnSlideRoll.BackColor = System.Drawing.Color.Transparent;
            this.btnSlideRoll.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnSlideRoll.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnSlideRoll.FlatAppearance.BorderSize = 0;
            this.btnSlideRoll.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSlideRoll.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSlideRoll.Image = global::AgOpenGPS.Properties.Resources.RollSlidehill_off;
            this.btnSlideRoll.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnSlideRoll.Location = new System.Drawing.Point(162, 76);
            this.btnSlideRoll.Name = "btnSlideRoll";
            this.btnSlideRoll.Size = new System.Drawing.Size(93, 61);
            this.btnSlideRoll.TabIndex = 601;
            this.btnSlideRoll.Text = "Slope";
            this.btnSlideRoll.UseVisualStyleBackColor = false;
            this.btnSlideRoll.Click += new System.EventHandler(this.btnSlideRoll_Click);
            // 
            // btnThirdAntenna
            // 
            this.btnThirdAntenna.BackColor = System.Drawing.Color.WhiteSmoke;
            this.btnThirdAntenna.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnThirdAntenna.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnThirdAntenna.FlatAppearance.BorderSize = 0;
            this.btnThirdAntenna.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnThirdAntenna.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnThirdAntenna.Image = global::AgOpenGPS.Properties.Resources.AntennaNoOffset;
            this.btnThirdAntenna.ImageAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnThirdAntenna.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnThirdAntenna.Location = new System.Drawing.Point(315, 68);
            this.btnThirdAntenna.Name = "btnThirdAntenna";
            this.btnThirdAntenna.Size = new System.Drawing.Size(93, 69);
            this.btnThirdAntenna.TabIndex = 602;
            this.btnThirdAntenna.Text = "Tool";
            this.btnThirdAntenna.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnThirdAntenna.UseVisualStyleBackColor = false;
            this.btnThirdAntenna.Click += new System.EventHandler(this.btnThirdAntenna_Click);
            // 
            // btnSlopeDirection
            // 
            this.btnSlopeDirection.BackColor = System.Drawing.Color.Chartreuse;
            this.btnSlopeDirection.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnSlopeDirection.FlatAppearance.BorderColor = System.Drawing.Color.DarkOrange;
            this.btnSlopeDirection.FlatAppearance.BorderSize = 0;
            this.btnSlopeDirection.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnSlopeDirection.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnSlopeDirection.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSlopeDirection.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnSlopeDirection.Image = global::AgOpenGPS.Properties.Resources.ArrowRight;
            this.btnSlopeDirection.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnSlopeDirection.Location = new System.Drawing.Point(163, 39);
            this.btnSlopeDirection.Name = "btnSlopeDirection";
            this.btnSlopeDirection.Size = new System.Drawing.Size(92, 31);
            this.btnSlopeDirection.TabIndex = 603;
            this.btnSlopeDirection.UseVisualStyleBackColor = false;
            this.btnSlopeDirection.Click += new System.EventHandler(this.btnSlopeDirection_Click);
            // 
            // btnAddnewSlope
            // 
            this.btnAddnewSlope.FlatAppearance.BorderColor = System.Drawing.Color.Blue;
            this.btnAddnewSlope.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnAddnewSlope.ForeColor = System.Drawing.Color.Blue;
            this.btnAddnewSlope.Image = global::AgOpenGPS.Properties.Resources.AddNew;
            this.btnAddnewSlope.Location = new System.Drawing.Point(12, 151);
            this.btnAddnewSlope.Name = "btnAddnewSlope";
            this.btnAddnewSlope.Size = new System.Drawing.Size(70, 70);
            this.btnAddnewSlope.TabIndex = 604;
            this.btnAddnewSlope.Text = "Slope";
            this.btnAddnewSlope.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnAddnewSlope.UseVisualStyleBackColor = false;
            this.btnAddnewSlope.Visible = false;
            this.btnAddnewSlope.Click += new System.EventHandler(this.btnAddnewSlope_Click);
            // 
            // nudSetToolAntHight1
            // 
            this.nudSetToolAntHight1.BackColor = System.Drawing.Color.White;
            this.nudSetToolAntHight1.Font = new System.Drawing.Font("Tahoma", 16F, System.Drawing.FontStyle.Bold);
            this.nudSetToolAntHight1.Location = new System.Drawing.Point(315, 192);
            this.nudSetToolAntHight1.Maximum = new decimal(new int[] {
            5000,
            0,
            0,
            0});
            this.nudSetToolAntHight1.Name = "nudSetToolAntHight1";
            this.nudSetToolAntHight1.ReadOnly = true;
            this.nudSetToolAntHight1.Size = new System.Drawing.Size(98, 33);
            this.nudSetToolAntHight1.TabIndex = 605;
            this.nudSetToolAntHight1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetToolAntHight1.ValueChanged += new System.EventHandler(this.nudSetToolAntHight1_ValueChanged);
            this.nudSetToolAntHight1.Click += new System.EventHandler(this.nudSetToolAntHight1_Click);
            // 
            // FormToolTracks
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.AutoValidate = System.Windows.Forms.AutoValidate.EnablePreventFocusChange;
            this.BackColor = System.Drawing.SystemColors.InactiveCaption;
            this.ClientSize = new System.Drawing.Size(470, 511);
            this.ControlBox = false;
            this.Controls.Add(this.nudSetToolAntHight1);
            this.Controls.Add(this.btnAddnewSlope);
            this.Controls.Add(this.btnSlopeDirection);
            this.Controls.Add(this.btnThirdAntenna);
            this.Controls.Add(this.btnSlideRoll);
            this.Controls.Add(this.nudToolbehindPivot);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.lblToolPWM);
            this.Controls.Add(this.lblToolLookAhead);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.hToolPWM);
            this.Controls.Add(this.hsbarToolLookAhead);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.btnSetupopenclose);
            this.Controls.Add(this.nudSetToolOffset);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.btnleaveMenue);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.btnToolAtWork);
            this.Controls.Add(this.btnToolTrackDelete);
            this.Controls.Add(this.btnExit);
            this.Controls.Add(this.btnAddToolTrackPts);
            this.Location = new System.Drawing.Point(139, 100);
            this.MaximizeBox = false;
            this.Name = "FormToolTracks";
            this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            this.Text = "Tool Antenna";
            this.Load += new System.EventHandler(this.FormToolTracks_Load);
            this.groupBox5.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.nudSetToolOffset)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudToolbehindPivot)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetToolAntHight1)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button btnAddToolTrackPts;
        private System.Windows.Forms.Button btnExit;
        private System.Windows.Forms.Button btnToolTrackDelete;
        private System.Windows.Forms.Button btnToolAtWork;
//        private NudlessNumericUpDown nudSetToolAntHight;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Button btnleaveMenue;
        private System.Windows.Forms.Label label4;
        private NudlessNumericUpDown nudSetToolOffset;
        private System.Windows.Forms.Button btnSetupopenclose;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.HScrollBar hsbarToolLookAhead;
        private System.Windows.Forms.HScrollBar hToolPWM;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label lblToolLookAhead;
        private System.Windows.Forms.Label lblToolPWM;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.RadioButton rbtnToolAntennaCenter;
        private System.Windows.Forms.RadioButton rbtnToolAntennaRight;
        private System.Windows.Forms.RadioButton rbtnToolAntennaLeft;
        private System.Windows.Forms.Label label3;
        private NudlessNumericUpDown nudToolbehindPivot;
        private System.Windows.Forms.Button btnSlideRoll;
        private System.Windows.Forms.Button btnThirdAntenna;
        private System.Windows.Forms.Button btnSlopeDirection;
        private System.Windows.Forms.Button btnAddnewSlope;
        private NudlessNumericUpDown nudSetToolAntHight1;
    }
}