namespace AgOpenGPS
{
    partial class FormFlags
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
            this.btnSouth = new ProXoft.WinForms.RepeatButton();
            this.btnNorth = new ProXoft.WinForms.RepeatButton();
            this.lblFlagSelected = new System.Windows.Forms.Label();
            this.lblLonStart = new System.Windows.Forms.Label();
            this.lblLatStart = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.lblEasting = new System.Windows.Forms.Label();
            this.lblNorthing = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.tboxFlagNotes = new System.Windows.Forms.TextBox();
            this.btnExit = new System.Windows.Forms.Button();
            this.btnDeleteFlag = new System.Windows.Forms.Button();
            this.lblHeading = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.label2 = new System.Windows.Forms.Label();
            this.lblDistanceToFlag = new System.Windows.Forms.Label();
            this.checkFlagAlarm = new System.Windows.Forms.CheckBox();
            this.lblFlagAlarm = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.nudSetDistanceFlag = new AgOpenGPS.NudlessNumericUpDown();
            this.btnYellow = new System.Windows.Forms.Button();
            this.btnGreen = new System.Windows.Forms.Button();
            this.btnRed = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceFlag)).BeginInit();
            this.SuspendLayout();
            // 
            // btnSouth
            // 
            this.btnSouth.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnSouth.FlatAppearance.BorderSize = 0;
            this.btnSouth.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSouth.Font = new System.Drawing.Font("Tahoma", 18F, System.Drawing.FontStyle.Bold);
            this.btnSouth.Image = global::AgOpenGPS.Properties.Resources.DnArrow64;
            this.btnSouth.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnSouth.Location = new System.Drawing.Point(197, 5);
            this.btnSouth.Name = "btnSouth";
            this.btnSouth.Size = new System.Drawing.Size(78, 59);
            this.btnSouth.TabIndex = 195;
            this.btnSouth.UseVisualStyleBackColor = true;
            this.btnSouth.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btnSouth_MouseDown);
            // 
            // btnNorth
            // 
            this.btnNorth.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnNorth.FlatAppearance.BorderSize = 0;
            this.btnNorth.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnNorth.Font = new System.Drawing.Font("Tahoma", 18F, System.Drawing.FontStyle.Bold);
            this.btnNorth.Image = global::AgOpenGPS.Properties.Resources.UpArrow64;
            this.btnNorth.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnNorth.Location = new System.Drawing.Point(100, 5);
            this.btnNorth.Name = "btnNorth";
            this.btnNorth.Size = new System.Drawing.Size(64, 59);
            this.btnNorth.TabIndex = 196;
            this.btnNorth.UseVisualStyleBackColor = true;
            this.btnNorth.MouseDown += new System.Windows.Forms.MouseEventHandler(this.btnNorth_MouseDown);
            // 
            // lblFlagSelected
            // 
            this.lblFlagSelected.AutoSize = true;
            this.lblFlagSelected.BackColor = System.Drawing.Color.Transparent;
            this.lblFlagSelected.Font = new System.Drawing.Font("Tahoma", 24F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFlagSelected.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.lblFlagSelected.Location = new System.Drawing.Point(5, 18);
            this.lblFlagSelected.Name = "lblFlagSelected";
            this.lblFlagSelected.Size = new System.Drawing.Size(57, 39);
            this.lblFlagSelected.TabIndex = 200;
            this.lblFlagSelected.Text = "99";
            // 
            // lblLonStart
            // 
            this.lblLonStart.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblLonStart.AutoSize = true;
            this.lblLonStart.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblLonStart.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.lblLonStart.Location = new System.Drawing.Point(185, 205);
            this.lblLonStart.Name = "lblLonStart";
            this.lblLonStart.Size = new System.Drawing.Size(101, 19);
            this.lblLonStart.TabIndex = 204;
            this.lblLonStart.Text = "-188.888888";
            // 
            // lblLatStart
            // 
            this.lblLatStart.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblLatStart.AutoSize = true;
            this.lblLatStart.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblLatStart.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.lblLatStart.Location = new System.Drawing.Point(40, 205);
            this.lblLatStart.Name = "lblLatStart";
            this.lblLatStart.Size = new System.Drawing.Size(92, 19);
            this.lblLatStart.TabIndex = 203;
            this.lblLatStart.Text = "-99.999999";
            // 
            // label4
            // 
            this.label4.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label4.Location = new System.Drawing.Point(149, 204);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(41, 19);
            this.label4.TabIndex = 202;
            this.label4.Text = "Lon:";
            // 
            // label3
            // 
            this.label3.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label3.Location = new System.Drawing.Point(8, 204);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(36, 19);
            this.label3.TabIndex = 201;
            this.label3.Text = "Lat:";
            // 
            // lblEasting
            // 
            this.lblEasting.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblEasting.AutoSize = true;
            this.lblEasting.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblEasting.ForeColor = System.Drawing.SystemColors.ControlText;
            this.lblEasting.Location = new System.Drawing.Point(133, 230);
            this.lblEasting.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.lblEasting.Name = "lblEasting";
            this.lblEasting.Size = new System.Drawing.Size(60, 19);
            this.lblEasting.TabIndex = 208;
            this.lblEasting.Text = "Easting";
            // 
            // lblNorthing
            // 
            this.lblNorthing.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblNorthing.AutoSize = true;
            this.lblNorthing.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblNorthing.ForeColor = System.Drawing.SystemColors.ControlText;
            this.lblNorthing.Location = new System.Drawing.Point(32, 230);
            this.lblNorthing.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.lblNorthing.Name = "lblNorthing";
            this.lblNorthing.Size = new System.Drawing.Size(71, 19);
            this.lblNorthing.TabIndex = 207;
            this.lblNorthing.Text = "Northing";
            // 
            // label5
            // 
            this.label5.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label5.Location = new System.Drawing.Point(114, 230);
            this.label5.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(24, 19);
            this.label5.TabIndex = 206;
            this.label5.Text = "E:";
            // 
            // label1
            // 
            this.label1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label1.Location = new System.Drawing.Point(11, 230);
            this.label1.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(26, 19);
            this.label1.TabIndex = 205;
            this.label1.Text = "N:";
            // 
            // tboxFlagNotes
            // 
            this.tboxFlagNotes.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tboxFlagNotes.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tboxFlagNotes.Location = new System.Drawing.Point(7, 70);
            this.tboxFlagNotes.Multiline = true;
            this.tboxFlagNotes.Name = "tboxFlagNotes";
            this.tboxFlagNotes.ScrollBars = System.Windows.Forms.ScrollBars.Horizontal;
            this.tboxFlagNotes.Size = new System.Drawing.Size(281, 35);
            this.tboxFlagNotes.TabIndex = 209;
            this.tboxFlagNotes.Text = "1";
            this.tboxFlagNotes.Click += new System.EventHandler(this.tboxFlagNotes_Click);
            this.tboxFlagNotes.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tboxFlagNotes_KeyPress);
            this.tboxFlagNotes.Leave += new System.EventHandler(this.tboxFlagNotes_Leave);
            // 
            // btnExit
            // 
            this.btnExit.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnExit.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnExit.Cursor = System.Windows.Forms.Cursors.Default;
            this.btnExit.FlatAppearance.BorderSize = 0;
            this.btnExit.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnExit.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnExit.Image = global::AgOpenGPS.Properties.Resources.OK64;
            this.btnExit.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnExit.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnExit.Location = new System.Drawing.Point(204, 315);
            this.btnExit.Name = "btnExit";
            this.btnExit.Size = new System.Drawing.Size(78, 66);
            this.btnExit.TabIndex = 210;
            this.btnExit.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnExit.UseVisualStyleBackColor = true;
            this.btnExit.Click += new System.EventHandler(this.btnExit_Click);
            // 
            // btnDeleteFlag
            // 
            this.btnDeleteFlag.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.btnDeleteFlag.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnDeleteFlag.FlatAppearance.BorderSize = 0;
            this.btnDeleteFlag.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnDeleteFlag.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.btnDeleteFlag.Image = global::AgOpenGPS.Properties.Resources.FlagDelete;
            this.btnDeleteFlag.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnDeleteFlag.Location = new System.Drawing.Point(7, 305);
            this.btnDeleteFlag.Name = "btnDeleteFlag";
            this.btnDeleteFlag.Size = new System.Drawing.Size(70, 68);
            this.btnDeleteFlag.TabIndex = 211;
            this.btnDeleteFlag.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnDeleteFlag.UseVisualStyleBackColor = true;
            this.btnDeleteFlag.Click += new System.EventHandler(this.btnDeleteFlag_Click);
            // 
            // lblHeading
            // 
            this.lblHeading.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblHeading.AutoSize = true;
            this.lblHeading.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblHeading.ForeColor = System.Drawing.SystemColors.ControlText;
            this.lblHeading.Location = new System.Drawing.Point(235, 230);
            this.lblHeading.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.lblHeading.Name = "lblHeading";
            this.lblHeading.Size = new System.Drawing.Size(50, 19);
            this.lblHeading.TabIndex = 214;
            this.lblHeading.Text = "359.8";
            // 
            // label6
            // 
            this.label6.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label6.Location = new System.Drawing.Point(216, 230);
            this.label6.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(26, 19);
            this.label6.TabIndex = 213;
            this.label6.Text = "H:";
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 1000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // label2
            // 
            this.label2.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label2.Location = new System.Drawing.Point(32, 177);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(132, 19);
            this.label2.TabIndex = 215;
            this.label2.Text = "Distance To Flag:";
            // 
            // lblDistanceToFlag
            // 
            this.lblDistanceToFlag.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblDistanceToFlag.AutoSize = true;
            this.lblDistanceToFlag.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDistanceToFlag.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.lblDistanceToFlag.Location = new System.Drawing.Point(160, 176);
            this.lblDistanceToFlag.Name = "lblDistanceToFlag";
            this.lblDistanceToFlag.Size = new System.Drawing.Size(120, 23);
            this.lblDistanceToFlag.TabIndex = 216;
            this.lblDistanceToFlag.Text = "-99.999999";
            // 
            // checkFlagAlarm
            // 
            this.checkFlagAlarm.AutoSize = true;
            this.checkFlagAlarm.Checked = true;
            this.checkFlagAlarm.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkFlagAlarm.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.checkFlagAlarm.Location = new System.Drawing.Point(137, 345);
            this.checkFlagAlarm.Name = "checkFlagAlarm";
            this.checkFlagAlarm.Size = new System.Drawing.Size(15, 14);
            this.checkFlagAlarm.TabIndex = 217;
            this.checkFlagAlarm.UseVisualStyleBackColor = true;
            this.checkFlagAlarm.CheckedChanged += new System.EventHandler(this.checkFlagAlarm_CheckedChanged);
            // 
            // lblFlagAlarm
            // 
            this.lblFlagAlarm.AutoSize = true;
            this.lblFlagAlarm.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFlagAlarm.Location = new System.Drawing.Point(114, 315);
            this.lblFlagAlarm.Name = "lblFlagAlarm";
            this.lblFlagAlarm.Size = new System.Drawing.Size(55, 20);
            this.lblFlagAlarm.TabIndex = 218;
            this.lblFlagAlarm.Text = "Alarm";
            // 
            // label7
            // 
            this.label7.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label7.Location = new System.Drawing.Point(38, 252);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(166, 19);
            this.label7.TabIndex = 220;
            this.label7.Text = "distance Flags to pivot";
            this.label7.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.label7.Click += new System.EventHandler(this.label7_Click);
            // 
            // label8
            // 
            this.label8.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label8.Location = new System.Drawing.Point(61, 271);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(121, 19);
            this.label8.TabIndex = 466;
            this.label8.Text = "-40 to 40 meter";
            this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // nudSetDistanceFlag
            // 
            this.nudSetDistanceFlag.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.nudSetDistanceFlag.BackColor = System.Drawing.Color.White;
            this.nudSetDistanceFlag.DecimalPlaces = 1;
            this.nudSetDistanceFlag.Font = new System.Drawing.Font("Tahoma", 20.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.nudSetDistanceFlag.Location = new System.Drawing.Point(204, 252);
            this.nudSetDistanceFlag.Maximum = new decimal(new int[] {
            40,
            0,
            0,
            0});
            this.nudSetDistanceFlag.Minimum = new decimal(new int[] {
            40,
            0,
            0,
            -2147483648});
            this.nudSetDistanceFlag.Name = "nudSetDistanceFlag";
            this.nudSetDistanceFlag.ReadOnly = true;
            this.nudSetDistanceFlag.Size = new System.Drawing.Size(84, 40);
            this.nudSetDistanceFlag.TabIndex = 465;
            this.nudSetDistanceFlag.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudSetDistanceFlag.Value = new decimal(new int[] {
            6,
            0,
            0,
            0});
            this.nudSetDistanceFlag.Click += new System.EventHandler(this.nudSetDistanceFlag_Click);
            // 
            // btnYellow
            // 
            this.btnYellow.BackColor = System.Drawing.Color.Transparent;
            this.btnYellow.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnYellow.FlatAppearance.BorderSize = 0;
            this.btnYellow.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnYellow.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnYellow.Image = global::AgOpenGPS.Properties.Resources.FlagYel;
            this.btnYellow.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnYellow.Location = new System.Drawing.Point(12, 111);
            this.btnYellow.Name = "btnYellow";
            this.btnYellow.Size = new System.Drawing.Size(71, 52);
            this.btnYellow.TabIndex = 467;
            this.btnYellow.Text = "+";
            this.btnYellow.TextAlign = System.Drawing.ContentAlignment.BottomRight;
            this.btnYellow.UseVisualStyleBackColor = false;
            this.btnYellow.Click += new System.EventHandler(this.btnRed_Click);
            // 
            // btnGreen
            // 
            this.btnGreen.BackColor = System.Drawing.Color.Transparent;
            this.btnGreen.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnGreen.FlatAppearance.BorderSize = 0;
            this.btnGreen.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnGreen.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnGreen.Image = global::AgOpenGPS.Properties.Resources.FlagGrn;
            this.btnGreen.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnGreen.Location = new System.Drawing.Point(102, 110);
            this.btnGreen.Name = "btnGreen";
            this.btnGreen.Size = new System.Drawing.Size(80, 58);
            this.btnGreen.TabIndex = 468;
            this.btnGreen.Text = "+";
            this.btnGreen.TextAlign = System.Drawing.ContentAlignment.BottomRight;
            this.btnGreen.UseVisualStyleBackColor = false;
            this.btnGreen.Click += new System.EventHandler(this.btnRed_Click);
            // 
            // btnRed
            // 
            this.btnRed.BackColor = System.Drawing.Color.Transparent;
            this.btnRed.FlatAppearance.BorderColor = System.Drawing.SystemColors.HotTrack;
            this.btnRed.FlatAppearance.BorderSize = 0;
            this.btnRed.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnRed.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnRed.Image = global::AgOpenGPS.Properties.Resources.FlagRed;
            this.btnRed.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.btnRed.Location = new System.Drawing.Point(197, 111);
            this.btnRed.Name = "btnRed";
            this.btnRed.Size = new System.Drawing.Size(83, 57);
            this.btnRed.TabIndex = 469;
            this.btnRed.Text = "+";
            this.btnRed.TextAlign = System.Drawing.ContentAlignment.BottomRight;
            this.btnRed.UseVisualStyleBackColor = false;
            this.btnRed.Click += new System.EventHandler(this.btnRed_Click);
            // 
            // FormFlags
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.WhiteSmoke;
            this.ClientSize = new System.Drawing.Size(300, 385);
            this.ControlBox = false;
            this.Controls.Add(this.btnRed);
            this.Controls.Add(this.btnGreen);
            this.Controls.Add(this.btnYellow);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.nudSetDistanceFlag);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.lblFlagAlarm);
            this.Controls.Add(this.checkFlagAlarm);
            this.Controls.Add(this.lblDistanceToFlag);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.lblHeading);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.btnDeleteFlag);
            this.Controls.Add(this.btnExit);
            this.Controls.Add(this.tboxFlagNotes);
            this.Controls.Add(this.lblEasting);
            this.Controls.Add(this.lblNorthing);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.lblLonStart);
            this.Controls.Add(this.lblLatStart);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.lblFlagSelected);
            this.Controls.Add(this.btnSouth);
            this.Controls.Add(this.btnNorth);
            this.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.SizableToolWindow;
            this.Name = "FormFlags";
            this.ShowInTaskbar = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent;
            this.Text = "mark Flags";
            this.Load += new System.EventHandler(this.FormFlags_Load);
            ((System.ComponentModel.ISupportInitialize)(this.nudSetDistanceFlag)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private ProXoft.WinForms.RepeatButton btnSouth;
        private ProXoft.WinForms.RepeatButton btnNorth;
        private System.Windows.Forms.Label lblFlagSelected;
        private System.Windows.Forms.Label lblLonStart;
        private System.Windows.Forms.Label lblLatStart;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label lblEasting;
        private System.Windows.Forms.Label lblNorthing;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox tboxFlagNotes;
        private System.Windows.Forms.Button btnExit;
        private System.Windows.Forms.Button btnDeleteFlag;
        private System.Windows.Forms.Label lblHeading;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label lblDistanceToFlag;
        private System.Windows.Forms.CheckBox checkFlagAlarm;
        private System.Windows.Forms.Label lblFlagAlarm;
        private NudlessNumericUpDown nudSetDistanceFlag;
        public System.Windows.Forms.Label label7;
        public System.Windows.Forms.Label label8;
        private System.Windows.Forms.Button btnYellow;
        private System.Windows.Forms.Button btnGreen;
        private System.Windows.Forms.Button btnRed;
    }
}