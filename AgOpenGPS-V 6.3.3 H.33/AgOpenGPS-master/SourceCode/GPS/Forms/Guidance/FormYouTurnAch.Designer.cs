namespace AgOpenGPS
{
    partial class FormYouTurnAch
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
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.oglSelfyt = new OpenTK.GLControl();
            this.tlp1yt = new System.Windows.Forms.TableLayoutPanel();
            this.btnCancelTouchyt = new System.Windows.Forms.Button();
            this.btnALengthyt = new System.Windows.Forms.Button();
            this.rbtnCurveyt = new System.Windows.Forms.RadioButton();
            this.rbtnLineyt = new System.Windows.Forms.RadioButton();
            this.btnAShrinkyt = new System.Windows.Forms.Button();
            this.cboxToolWidthsyt = new System.Windows.Forms.ComboBox();
            this.btnExityt = new System.Windows.Forms.Button();
            this.btnBndLoopyt = new System.Windows.Forms.Button();
            this.cboxIsZoomyt = new System.Windows.Forms.CheckBox();
            this.cboxIsSectionControlled = new System.Windows.Forms.CheckBox();
            this.btnHeadlandOffyt = new System.Windows.Forms.Button();
            this.nudSetDistanceyt = new AgOpenGPS.NudlessNumericUpDown();
            this.btnDeleteCurveyt = new System.Windows.Forms.Button();
            this.btnDeleteYouturnyt = new System.Windows.Forms.Button();
            this.lblToolWidth = new System.Windows.Forms.Label();
            this.btnCycleBackwardyt = new System.Windows.Forms.Button();
            this.btnCycleForwardyt = new System.Windows.Forms.Button();
            this.btnBShrinkyt = new System.Windows.Forms.Button();
            this.btnBLengthyt = new System.Windows.Forms.Button();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.tlp1yt.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceyt)).BeginInit();
            this.SuspendLayout();
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 500;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // oglSelfyt
            // 
            this.oglSelfyt.BackColor = System.Drawing.Color.Black;
            this.oglSelfyt.Cursor = System.Windows.Forms.Cursors.Cross;
            this.oglSelfyt.ForeColor = System.Drawing.Color.Black;
            this.oglSelfyt.Location = new System.Drawing.Point(0, 1);
            this.oglSelfyt.Margin = new System.Windows.Forms.Padding(0);
            this.oglSelfyt.Name = "oglSelfyt";
            this.oglSelfyt.Size = new System.Drawing.Size(700, 700);
            this.oglSelfyt.TabIndex = 183;
            this.oglSelfyt.VSync = false;
            this.oglSelfyt.Load += new System.EventHandler(this.oglSelfyt_Load);
            this.oglSelfyt.Paint += new System.Windows.Forms.PaintEventHandler(this.oglSelfyt_Paint);
            this.oglSelfyt.MouseDown += new System.Windows.Forms.MouseEventHandler(this.oglSelfyt_MouseDown);
            this.oglSelfyt.Resize += new System.EventHandler(this.oglSelfyt_Resize);
            // 
            // tlp1yt
            // 
            this.tlp1yt.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)));
            this.tlp1yt.ColumnCount = 6;
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66668F));
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tlp1yt.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tlp1yt.Controls.Add(this.btnCancelTouchyt, 4, 1);
            this.tlp1yt.Controls.Add(this.btnALengthyt, 0, 1);
            this.tlp1yt.Controls.Add(this.rbtnCurveyt, 0, 2);
            this.tlp1yt.Controls.Add(this.rbtnLineyt, 3, 2);
            this.tlp1yt.Controls.Add(this.btnAShrinkyt, 2, 1);
            this.tlp1yt.Controls.Add(this.cboxToolWidthsyt, 4, 3);
            this.tlp1yt.Controls.Add(this.btnExityt, 4, 7);
            this.tlp1yt.Controls.Add(this.btnBndLoopyt, 4, 5);
            this.tlp1yt.Controls.Add(this.cboxIsZoomyt, 0, 5);
            this.tlp1yt.Controls.Add(this.cboxIsSectionControlled, 4, 0);
            this.tlp1yt.Controls.Add(this.btnHeadlandOffyt, 0, 7);
            this.tlp1yt.Controls.Add(this.nudSetDistanceyt, 0, 3);
            this.tlp1yt.Controls.Add(this.btnDeleteCurveyt, 2, 7);
            this.tlp1yt.Controls.Add(this.btnDeleteYouturnyt, 0, 6);
            this.tlp1yt.Controls.Add(this.lblToolWidth, 0, 4);
            this.tlp1yt.Controls.Add(this.btnCycleBackwardyt, 2, 6);
            this.tlp1yt.Controls.Add(this.btnCycleForwardyt, 4, 6);
            this.tlp1yt.Controls.Add(this.btnBShrinkyt, 2, 0);
            this.tlp1yt.Controls.Add(this.btnBLengthyt, 0, 0);
            this.tlp1yt.Location = new System.Drawing.Point(703, 2);
            this.tlp1yt.Name = "tlp1yt";
            this.tlp1yt.RowCount = 8;
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 12.41873F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 12.36926F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 14.1155F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.20509F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.238742F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 15.86175F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 14.26102F));
            this.tlp1yt.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 14.52991F));
            this.tlp1yt.Size = new System.Drawing.Size(299, 699);
            this.tlp1yt.TabIndex = 566;
            this.tlp1yt.Paint += new System.Windows.Forms.PaintEventHandler(this.tlp1yt_Paint);
            // 
            // btnCancelTouchyt
            // 
            this.btnCancelTouchyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCancelTouchyt.BackColor = System.Drawing.Color.Transparent;
            this.btnCancelTouchyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnCancelTouchyt, 2);
            this.btnCancelTouchyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCancelTouchyt.FlatAppearance.BorderSize = 0;
            this.btnCancelTouchyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCancelTouchyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCancelTouchyt.Image = global::AgOpenGPS.Properties.Resources.HeadlandDeletePoints;
            this.btnCancelTouchyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCancelTouchyt.Location = new System.Drawing.Point(211, 100);
            this.btnCancelTouchyt.Name = "btnCancelTouchyt";
            this.btnCancelTouchyt.Size = new System.Drawing.Size(72, 58);
            this.btnCancelTouchyt.TabIndex = 565;
            this.btnCancelTouchyt.UseVisualStyleBackColor = false;
            this.btnCancelTouchyt.Click += new System.EventHandler(this.btnCancelTouchyt_Click);
            // 
            // btnALengthyt
            // 
            this.btnALengthyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnALengthyt.BackColor = System.Drawing.Color.Transparent;
            this.btnALengthyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnALengthyt, 2);
            this.btnALengthyt.FlatAppearance.BorderColor = System.Drawing.Color.DarkOrange;
            this.btnALengthyt.FlatAppearance.BorderSize = 0;
            this.btnALengthyt.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnALengthyt.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnALengthyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnALengthyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnALengthyt.Image = global::AgOpenGPS.Properties.Resources.APlusPlusA;
            this.btnALengthyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnALengthyt.Location = new System.Drawing.Point(3, 103);
            this.btnALengthyt.Name = "btnALengthyt";
            this.btnALengthyt.Size = new System.Drawing.Size(92, 52);
            this.btnALengthyt.TabIndex = 352;
            this.btnALengthyt.UseVisualStyleBackColor = false;
            this.btnALengthyt.Click += new System.EventHandler(this.btnALengthyt_Click);
            // 
            // rbtnCurveyt
            // 
            this.rbtnCurveyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.rbtnCurveyt.Appearance = System.Windows.Forms.Appearance.Button;
            this.rbtnCurveyt.BackColor = System.Drawing.Color.AliceBlue;
            this.rbtnCurveyt.Checked = true;
            this.tlp1yt.SetColumnSpan(this.rbtnCurveyt, 3);
            this.rbtnCurveyt.FlatAppearance.CheckedBackColor = System.Drawing.Color.PaleTurquoise;
            this.rbtnCurveyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.rbtnCurveyt.Font = new System.Drawing.Font("Tahoma", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.rbtnCurveyt.ForeColor = System.Drawing.Color.Black;
            this.rbtnCurveyt.Image = global::AgOpenGPS.Properties.Resources.ABTrackCurve;
            this.rbtnCurveyt.Location = new System.Drawing.Point(13, 185);
            this.rbtnCurveyt.Name = "rbtnCurveyt";
            this.rbtnCurveyt.Size = new System.Drawing.Size(120, 72);
            this.rbtnCurveyt.TabIndex = 0;
            this.rbtnCurveyt.TabStop = true;
            this.rbtnCurveyt.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.rbtnCurveyt.UseVisualStyleBackColor = false;
            // 
            // rbtnLineyt
            // 
            this.rbtnLineyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.rbtnLineyt.Appearance = System.Windows.Forms.Appearance.Button;
            this.rbtnLineyt.BackColor = System.Drawing.Color.AliceBlue;
            this.tlp1yt.SetColumnSpan(this.rbtnLineyt, 3);
            this.rbtnLineyt.FlatAppearance.CheckedBackColor = System.Drawing.Color.PaleTurquoise;
            this.rbtnLineyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.rbtnLineyt.Font = new System.Drawing.Font("Tahoma", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.rbtnLineyt.ForeColor = System.Drawing.Color.Black;
            this.rbtnLineyt.Image = global::AgOpenGPS.Properties.Resources.ABTrackAB;
            this.rbtnLineyt.Location = new System.Drawing.Point(163, 187);
            this.rbtnLineyt.Name = "rbtnLineyt";
            this.rbtnLineyt.Size = new System.Drawing.Size(120, 68);
            this.rbtnLineyt.TabIndex = 2;
            this.rbtnLineyt.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.rbtnLineyt.UseVisualStyleBackColor = false;
            this.rbtnLineyt.CheckedChanged += new System.EventHandler(this.rbtnLineyt_CheckedChanged);
            // 
            // btnAShrinkyt
            // 
            this.btnAShrinkyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnAShrinkyt.BackColor = System.Drawing.Color.Transparent;
            this.btnAShrinkyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnAShrinkyt, 2);
            this.btnAShrinkyt.FlatAppearance.BorderColor = System.Drawing.Color.DarkOrange;
            this.btnAShrinkyt.FlatAppearance.BorderSize = 0;
            this.btnAShrinkyt.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnAShrinkyt.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnAShrinkyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnAShrinkyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnAShrinkyt.Image = global::AgOpenGPS.Properties.Resources.APlusMinusA;
            this.btnAShrinkyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnAShrinkyt.Location = new System.Drawing.Point(101, 103);
            this.btnAShrinkyt.Name = "btnAShrinkyt";
            this.btnAShrinkyt.Size = new System.Drawing.Size(92, 52);
            this.btnAShrinkyt.TabIndex = 525;
            this.btnAShrinkyt.UseVisualStyleBackColor = false;
            this.btnAShrinkyt.Click += new System.EventHandler(this.btnAShrinkyt_Click);
            // 
            // cboxToolWidthsyt
            // 
            this.cboxToolWidthsyt.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.cboxToolWidthsyt.BackColor = System.Drawing.Color.Lavender;
            this.tlp1yt.SetColumnSpan(this.cboxToolWidthsyt, 2);
            this.cboxToolWidthsyt.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboxToolWidthsyt.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.cboxToolWidthsyt.Font = new System.Drawing.Font("Tahoma", 27.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxToolWidthsyt.FormattingEnabled = true;
            this.cboxToolWidthsyt.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10"});
            this.cboxToolWidthsyt.Location = new System.Drawing.Point(198, 292);
            this.cboxToolWidthsyt.Margin = new System.Windows.Forms.Padding(2, 3, 2, 3);
            this.cboxToolWidthsyt.Name = "cboxToolWidthsyt";
            this.cboxToolWidthsyt.Size = new System.Drawing.Size(90, 53);
            this.cboxToolWidthsyt.TabIndex = 510;
            this.cboxToolWidthsyt.SelectedIndexChanged += new System.EventHandler(this.cboxToolWidthsyt_SelectedIndexChanged);
            // 
            // btnExityt
            // 
            this.btnExityt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnExityt.BackColor = System.Drawing.Color.Transparent;
            this.tlp1yt.SetColumnSpan(this.btnExityt, 2);
            this.btnExityt.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.btnExityt.FlatAppearance.BorderSize = 0;
            this.btnExityt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnExityt.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnExityt.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(192)))), ((int)(((byte)(64)))), ((int)(((byte)(0)))));
            this.btnExityt.Image = global::AgOpenGPS.Properties.Resources.OK64;
            this.btnExityt.ImageAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.btnExityt.Location = new System.Drawing.Point(211, 610);
            this.btnExityt.Name = "btnExityt";
            this.btnExityt.Size = new System.Drawing.Size(72, 71);
            this.btnExityt.TabIndex = 0;
            this.btnExityt.UseVisualStyleBackColor = false;
            this.btnExityt.Click += new System.EventHandler(this.btnExityt_Click);
            // 
            // btnBndLoopyt
            // 
            this.btnBndLoopyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnBndLoopyt.BackColor = System.Drawing.Color.Transparent;
            this.btnBndLoopyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.tlp1yt.SetColumnSpan(this.btnBndLoopyt, 2);
            this.btnBndLoopyt.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnBndLoopyt.FlatAppearance.BorderSize = 0;
            this.btnBndLoopyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnBndLoopyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnBndLoopyt.Image = global::AgOpenGPS.Properties.Resources.HeadlandBuild1;
            this.btnBndLoopyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnBndLoopyt.Location = new System.Drawing.Point(216, 404);
            this.btnBndLoopyt.Name = "btnBndLoopyt";
            this.btnBndLoopyt.Size = new System.Drawing.Size(63, 70);
            this.btnBndLoopyt.TabIndex = 504;
            this.btnBndLoopyt.UseVisualStyleBackColor = false;
            this.btnBndLoopyt.Click += new System.EventHandler(this.btnBndLoopyt_Click);
            // 
            // cboxIsZoomyt
            // 
            this.cboxIsZoomyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.cboxIsZoomyt.Appearance = System.Windows.Forms.Appearance.Button;
            this.cboxIsZoomyt.BackColor = System.Drawing.Color.WhiteSmoke;
            this.tlp1yt.SetColumnSpan(this.cboxIsZoomyt, 2);
            this.cboxIsZoomyt.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.cboxIsZoomyt.FlatAppearance.CheckedBackColor = System.Drawing.Color.FromArgb(((int)(((byte)(180)))), ((int)(((byte)(255)))), ((int)(((byte)(160)))));
            this.cboxIsZoomyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.cboxIsZoomyt.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxIsZoomyt.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.cboxIsZoomyt.Image = global::AgOpenGPS.Properties.Resources.ZoomOGL;
            this.cboxIsZoomyt.Location = new System.Drawing.Point(3, 407);
            this.cboxIsZoomyt.Name = "cboxIsZoomyt";
            this.cboxIsZoomyt.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.cboxIsZoomyt.Size = new System.Drawing.Size(92, 63);
            this.cboxIsZoomyt.TabIndex = 564;
            this.cboxIsZoomyt.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.cboxIsZoomyt.UseVisualStyleBackColor = false;
            this.cboxIsZoomyt.CheckedChanged += new System.EventHandler(this.cboxIsZoomyt_CheckedChanged);
            // 
            // cboxIsSectionControlled
            // 
            this.cboxIsSectionControlled.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.cboxIsSectionControlled.Appearance = System.Windows.Forms.Appearance.Button;
            this.cboxIsSectionControlled.BackColor = System.Drawing.Color.Transparent;
            this.cboxIsSectionControlled.Checked = true;
            this.cboxIsSectionControlled.CheckState = System.Windows.Forms.CheckState.Checked;
            this.tlp1yt.SetColumnSpan(this.cboxIsSectionControlled, 2);
            this.cboxIsSectionControlled.FlatAppearance.BorderColor = System.Drawing.Color.Silver;
            this.cboxIsSectionControlled.FlatAppearance.BorderSize = 0;
            this.cboxIsSectionControlled.FlatAppearance.CheckedBackColor = System.Drawing.Color.Transparent;
            this.cboxIsSectionControlled.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.cboxIsSectionControlled.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxIsSectionControlled.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.cboxIsSectionControlled.Image = global::AgOpenGPS.Properties.Resources.HeadlandSectionOff;
            this.cboxIsSectionControlled.Location = new System.Drawing.Point(216, 10);
            this.cboxIsSectionControlled.Name = "cboxIsSectionControlled";
            this.cboxIsSectionControlled.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.cboxIsSectionControlled.Size = new System.Drawing.Size(63, 65);
            this.cboxIsSectionControlled.TabIndex = 467;
            this.cboxIsSectionControlled.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.cboxIsSectionControlled.UseVisualStyleBackColor = false;
            // 
            // btnHeadlandOffyt
            // 
            this.btnHeadlandOffyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnHeadlandOffyt.BackColor = System.Drawing.Color.Transparent;
            this.btnHeadlandOffyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.tlp1yt.SetColumnSpan(this.btnHeadlandOffyt, 2);
            this.btnHeadlandOffyt.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnHeadlandOffyt.FlatAppearance.BorderSize = 0;
            this.btnHeadlandOffyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnHeadlandOffyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnHeadlandOffyt.Image = global::AgOpenGPS.Properties.Resources.SwitchOff;
            this.btnHeadlandOffyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnHeadlandOffyt.Location = new System.Drawing.Point(17, 610);
            this.btnHeadlandOffyt.Name = "btnHeadlandOffyt";
            this.btnHeadlandOffyt.Size = new System.Drawing.Size(63, 71);
            this.btnHeadlandOffyt.TabIndex = 519;
            this.btnHeadlandOffyt.UseVisualStyleBackColor = false;
            this.btnHeadlandOffyt.Click += new System.EventHandler(this.btnHeadlandOffyt_Click);
            // 
            // nudSetDistanceyt
            // 
            this.nudSetDistanceyt.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.nudSetDistanceyt.BackColor = System.Drawing.Color.White;
            this.tlp1yt.SetColumnSpan(this.nudSetDistanceyt, 4);
            this.nudSetDistanceyt.DecimalPlaces = 1;
            this.nudSetDistanceyt.Font = new System.Drawing.Font("Tahoma", 27.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.nudSetDistanceyt.Location = new System.Drawing.Point(3, 293);
            this.nudSetDistanceyt.Maximum = new decimal(new int[] {
            40,
            0,
            0,
            0});
            this.nudSetDistanceyt.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            -2147483648});
            this.nudSetDistanceyt.Name = "nudSetDistanceyt";
            this.nudSetDistanceyt.ReadOnly = true;
            this.nudSetDistanceyt.Size = new System.Drawing.Size(142, 52);
            this.nudSetDistanceyt.TabIndex = 464;
            this.nudSetDistanceyt.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetDistanceyt.Click += new System.EventHandler(this.nudSetDistanceyt_Click);
            // 
            // btnDeleteCurveyt
            // 
            this.btnDeleteCurveyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnDeleteCurveyt.BackColor = System.Drawing.Color.Transparent;
            this.btnDeleteCurveyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnDeleteCurveyt, 2);
            this.btnDeleteCurveyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnDeleteCurveyt.FlatAppearance.BorderSize = 0;
            this.btnDeleteCurveyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnDeleteCurveyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnDeleteCurveyt.Image = global::AgOpenGPS.Properties.Resources.Trash;
            this.btnDeleteCurveyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnDeleteCurveyt.Location = new System.Drawing.Point(118, 618);
            this.btnDeleteCurveyt.Name = "btnDeleteCurveyt";
            this.btnDeleteCurveyt.Size = new System.Drawing.Size(57, 55);
            this.btnDeleteCurveyt.TabIndex = 6;
            this.btnDeleteCurveyt.UseVisualStyleBackColor = false;
            this.btnDeleteCurveyt.Click += new System.EventHandler(this.btnDeleteCurveyt_Click);
            // 
            // btnDeleteYouturnyt
            // 
            this.btnDeleteYouturnyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnDeleteYouturnyt.BackColor = System.Drawing.Color.Transparent;
            this.btnDeleteYouturnyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.tlp1yt.SetColumnSpan(this.btnDeleteYouturnyt, 2);
            this.btnDeleteYouturnyt.FlatAppearance.BorderColor = System.Drawing.Color.Black;
            this.btnDeleteYouturnyt.FlatAppearance.BorderSize = 0;
            this.btnDeleteYouturnyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnDeleteYouturnyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnDeleteYouturnyt.Image = global::AgOpenGPS.Properties.Resources.HeadlandReset;
            this.btnDeleteYouturnyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnDeleteYouturnyt.Location = new System.Drawing.Point(17, 510);
            this.btnDeleteYouturnyt.Name = "btnDeleteYouturnyt";
            this.btnDeleteYouturnyt.Size = new System.Drawing.Size(63, 67);
            this.btnDeleteYouturnyt.TabIndex = 465;
            this.btnDeleteYouturnyt.UseVisualStyleBackColor = false;
            this.btnDeleteYouturnyt.Click += new System.EventHandler(this.btnDeleteYouturnyt_Click);
            // 
            // lblToolWidth
            // 
            this.lblToolWidth.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.tlp1yt.SetColumnSpan(this.lblToolWidth, 6);
            this.lblToolWidth.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblToolWidth.ForeColor = System.Drawing.Color.Black;
            this.lblToolWidth.Location = new System.Drawing.Point(3, 353);
            this.lblToolWidth.Name = "lblToolWidth";
            this.lblToolWidth.Size = new System.Drawing.Size(293, 26);
            this.lblToolWidth.TabIndex = 511;
            this.lblToolWidth.Text = "3.86";
            this.lblToolWidth.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnCycleBackwardyt
            // 
            this.btnCycleBackwardyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleBackwardyt.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleBackwardyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnCycleBackwardyt, 2);
            this.btnCycleBackwardyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleBackwardyt.FlatAppearance.BorderSize = 0;
            this.btnCycleBackwardyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleBackwardyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleBackwardyt.Image = global::AgOpenGPS.Properties.Resources.ABLineCycleBk;
            this.btnCycleBackwardyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleBackwardyt.Location = new System.Drawing.Point(115, 509);
            this.btnCycleBackwardyt.Name = "btnCycleBackwardyt";
            this.btnCycleBackwardyt.Size = new System.Drawing.Size(63, 68);
            this.btnCycleBackwardyt.TabIndex = 507;
            this.btnCycleBackwardyt.UseVisualStyleBackColor = false;
            this.btnCycleBackwardyt.Click += new System.EventHandler(this.btnCycleBackwardyt_Click);
            // 
            // btnCycleForwardyt
            // 
            this.btnCycleForwardyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnCycleForwardyt.BackColor = System.Drawing.Color.Transparent;
            this.btnCycleForwardyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnCycleForwardyt, 2);
            this.btnCycleForwardyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnCycleForwardyt.FlatAppearance.BorderSize = 0;
            this.btnCycleForwardyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCycleForwardyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnCycleForwardyt.Image = global::AgOpenGPS.Properties.Resources.ABLineCycle;
            this.btnCycleForwardyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnCycleForwardyt.Location = new System.Drawing.Point(216, 509);
            this.btnCycleForwardyt.Name = "btnCycleForwardyt";
            this.btnCycleForwardyt.Size = new System.Drawing.Size(63, 68);
            this.btnCycleForwardyt.TabIndex = 5;
            this.btnCycleForwardyt.UseVisualStyleBackColor = false;
            this.btnCycleForwardyt.Click += new System.EventHandler(this.btnCycleForwardyt_Click);
            // 
            // btnBShrinkyt
            // 
            this.btnBShrinkyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnBShrinkyt.BackColor = System.Drawing.Color.Transparent;
            this.btnBShrinkyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnBShrinkyt, 2);
            this.btnBShrinkyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnBShrinkyt.FlatAppearance.BorderSize = 0;
            this.btnBShrinkyt.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnBShrinkyt.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnBShrinkyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnBShrinkyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnBShrinkyt.Image = global::AgOpenGPS.Properties.Resources.APlusMinusB;
            this.btnBShrinkyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnBShrinkyt.Location = new System.Drawing.Point(101, 17);
            this.btnBShrinkyt.Name = "btnBShrinkyt";
            this.btnBShrinkyt.Size = new System.Drawing.Size(92, 52);
            this.btnBShrinkyt.TabIndex = 524;
            this.btnBShrinkyt.UseVisualStyleBackColor = false;
            this.btnBShrinkyt.Click += new System.EventHandler(this.btnBShrinkyt_Click);
            // 
            // btnBLengthyt
            // 
            this.btnBLengthyt.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnBLengthyt.BackColor = System.Drawing.Color.Transparent;
            this.btnBLengthyt.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.tlp1yt.SetColumnSpan(this.btnBLengthyt, 2);
            this.btnBLengthyt.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnBLengthyt.FlatAppearance.BorderSize = 0;
            this.btnBLengthyt.FlatAppearance.MouseDownBackColor = System.Drawing.Color.Transparent;
            this.btnBLengthyt.FlatAppearance.MouseOverBackColor = System.Drawing.Color.Transparent;
            this.btnBLengthyt.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnBLengthyt.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnBLengthyt.Image = global::AgOpenGPS.Properties.Resources.APlusPlusB;
            this.btnBLengthyt.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnBLengthyt.Location = new System.Drawing.Point(3, 17);
            this.btnBLengthyt.Name = "btnBLengthyt";
            this.btnBLengthyt.Size = new System.Drawing.Size(92, 52);
            this.btnBLengthyt.TabIndex = 351;
            this.btnBLengthyt.UseVisualStyleBackColor = false;
            this.btnBLengthyt.Click += new System.EventHandler(this.btnBLengthyt_Click);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(1008, 0);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(100, 20);
            this.textBox1.TabIndex = 567;
            // 
            // FormYouTurnAch
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.WhiteSmoke;
            this.ClientSize = new System.Drawing.Size(1006, 726);
            this.ControlBox = false;
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.tlp1yt);
            this.Controls.Add(this.oglSelfyt);
            this.ForeColor = System.Drawing.Color.Black;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.MinimumSize = new System.Drawing.Size(1022, 675);
            this.Name = "FormYouTurnAch";
            this.ShowInTaskbar = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent;
            this.Text = "Click 2 points on the Boundary to Begin";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.FormYouTurnLine_FormClosing);
            this.Load += new System.EventHandler(this.FormYouTurnAch_Load);
            this.tlp1yt.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceyt)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Timer timer1;
        private OpenTK.GLControl oglSelfyt;
        private System.Windows.Forms.TableLayoutPanel tlp1yt;
        private System.Windows.Forms.Button btnCancelTouchyt;
        private System.Windows.Forms.Button btnALengthyt;
        private System.Windows.Forms.Button btnBLengthyt;
        private System.Windows.Forms.RadioButton rbtnCurveyt;
        private System.Windows.Forms.RadioButton rbtnLineyt;
        private System.Windows.Forms.Button btnBShrinkyt;
        private System.Windows.Forms.Button btnAShrinkyt;
        private System.Windows.Forms.ComboBox cboxToolWidthsyt;
        private System.Windows.Forms.Button btnExityt;
        private System.Windows.Forms.Button btnBndLoopyt;
        private System.Windows.Forms.CheckBox cboxIsZoomyt;
        private System.Windows.Forms.CheckBox cboxIsSectionControlled;
        private System.Windows.Forms.Button btnHeadlandOffyt;
        private NudlessNumericUpDown nudSetDistanceyt;
        private System.Windows.Forms.Button btnDeleteCurveyt;
        private System.Windows.Forms.Button btnDeleteYouturnyt;
        private System.Windows.Forms.Label lblToolWidth;
        private System.Windows.Forms.Button btnCycleBackwardyt;
        private System.Windows.Forms.Button btnCycleForwardyt;
        private System.Windows.Forms.TextBox textBox1;
    }
}