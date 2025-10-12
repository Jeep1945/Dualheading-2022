
namespace AgOpenGPS
{
    partial class FormYouTurn
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
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.cboxpRowWidthA = new System.Windows.Forms.ComboBox();
            this.cboxpRowWidthB = new System.Windows.Forms.ComboBox();
            this.btnFormYouTurnAch = new System.Windows.Forms.Button();
            this.btnturncircleA = new System.Windows.Forms.Button();
            this.btnturncircleB = new System.Windows.Forms.Button();
            this.btnYouTurnB = new System.Windows.Forms.Button();
            this.btnYouTurnA = new System.Windows.Forms.Button();
            this.btnExit = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 20.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(17, 9);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(108, 31);
            this.label2.TabIndex = 471;
            this.label2.Text = "Point A";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 20.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(167, 9);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(108, 31);
            this.label3.TabIndex = 472;
            this.label3.Text = "Point B";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Tahoma", 18F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.Location = new System.Drawing.Point(109, 209);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(61, 29);
            this.label5.TabIndex = 557;
            this.label5.Text = "skip";
            // 
            // cboxpRowWidthA
            // 
            this.cboxpRowWidthA.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.cboxpRowWidthA.BackColor = System.Drawing.Color.Lavender;
            this.cboxpRowWidthA.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboxpRowWidthA.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.cboxpRowWidthA.Font = new System.Drawing.Font("Tahoma", 24F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxpRowWidthA.FormattingEnabled = true;
            this.cboxpRowWidthA.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9"});
            this.cboxpRowWidthA.Location = new System.Drawing.Point(30, 198);
            this.cboxpRowWidthA.Margin = new System.Windows.Forms.Padding(2, 3, 2, 3);
            this.cboxpRowWidthA.Name = "cboxpRowWidthA";
            this.cboxpRowWidthA.Size = new System.Drawing.Size(71, 47);
            this.cboxpRowWidthA.TabIndex = 473;
            this.cboxpRowWidthA.SelectedIndexChanged += new System.EventHandler(this.cboxpRowWidthA_SelectedIndexChanged);
            // 
            // cboxpRowWidthB
            // 
            this.cboxpRowWidthB.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.cboxpRowWidthB.BackColor = System.Drawing.Color.Lavender;
            this.cboxpRowWidthB.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboxpRowWidthB.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.cboxpRowWidthB.Font = new System.Drawing.Font("Tahoma", 24F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxpRowWidthB.FormattingEnabled = true;
            this.cboxpRowWidthB.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9"});
            this.cboxpRowWidthB.Location = new System.Drawing.Point(178, 198);
            this.cboxpRowWidthB.Margin = new System.Windows.Forms.Padding(2, 3, 2, 3);
            this.cboxpRowWidthB.Name = "cboxpRowWidthB";
            this.cboxpRowWidthB.Size = new System.Drawing.Size(72, 47);
            this.cboxpRowWidthB.TabIndex = 474;
            this.cboxpRowWidthB.SelectedIndexChanged += new System.EventHandler(this.cboxpRowWidthB_SelectedIndexChanged);
            // 
            // btnFormYouTurnAch
            // 
            this.btnFormYouTurnAch.BackColor = System.Drawing.Color.GreenYellow;
            this.btnFormYouTurnAch.Font = new System.Drawing.Font("Tahoma", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnFormYouTurnAch.Location = new System.Drawing.Point(11, 267);
            this.btnFormYouTurnAch.Margin = new System.Windows.Forms.Padding(2);
            this.btnFormYouTurnAch.Name = "btnFormYouTurnAch";
            this.btnFormYouTurnAch.Size = new System.Drawing.Size(256, 63);
            this.btnFormYouTurnAch.TabIndex = 558;
            this.btnFormYouTurnAch.Text = "Create turnline";
            this.btnFormYouTurnAch.UseVisualStyleBackColor = false;
            this.btnFormYouTurnAch.Click += new System.EventHandler(this.btnFormYouTurnAch_Click);
            // 
            // btnturncircleA
            // 
            this.btnturncircleA.BackColor = System.Drawing.Color.Lime;
            this.btnturncircleA.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.btnturncircleA.Cursor = System.Windows.Forms.Cursors.Default;
            this.btnturncircleA.Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnturncircleA.Image = global::AgOpenGPS.Properties.Resources.TurnCurve_off;
            this.btnturncircleA.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnturncircleA.Location = new System.Drawing.Point(32, 122);
            this.btnturncircleA.Margin = new System.Windows.Forms.Padding(2);
            this.btnturncircleA.Name = "btnturncircleA";
            this.btnturncircleA.Size = new System.Drawing.Size(68, 64);
            this.btnturncircleA.TabIndex = 560;
            this.btnturncircleA.Text = "A";
            this.btnturncircleA.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnturncircleA.UseVisualStyleBackColor = false;
            this.btnturncircleA.Click += new System.EventHandler(this.btnturncircleA_Click);
            // 
            // btnturncircleB
            // 
            this.btnturncircleB.BackColor = System.Drawing.Color.Red;
            this.btnturncircleB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.btnturncircleB.Cursor = System.Windows.Forms.Cursors.Default;
            this.btnturncircleB.Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnturncircleB.Image = global::AgOpenGPS.Properties.Resources.TurnCurve_off;
            this.btnturncircleB.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnturncircleB.Location = new System.Drawing.Point(182, 123);
            this.btnturncircleB.Margin = new System.Windows.Forms.Padding(2);
            this.btnturncircleB.Name = "btnturncircleB";
            this.btnturncircleB.Size = new System.Drawing.Size(68, 63);
            this.btnturncircleB.TabIndex = 559;
            this.btnturncircleB.Text = "B";
            this.btnturncircleB.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnturncircleB.UseVisualStyleBackColor = false;
            this.btnturncircleB.Click += new System.EventHandler(this.btnturncircleB_Click);
            // 
            // btnYouTurnB
            // 
            this.btnYouTurnB.BackColor = System.Drawing.Color.Red;
            this.btnYouTurnB.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.btnYouTurnB.Cursor = System.Windows.Forms.Cursors.Default;
            this.btnYouTurnB.Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnYouTurnB.Image = global::AgOpenGPS.Properties.Resources.ArrowLeft;
            this.btnYouTurnB.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnYouTurnB.Location = new System.Drawing.Point(182, 54);
            this.btnYouTurnB.Margin = new System.Windows.Forms.Padding(2);
            this.btnYouTurnB.Name = "btnYouTurnB";
            this.btnYouTurnB.Size = new System.Drawing.Size(68, 64);
            this.btnYouTurnB.TabIndex = 553;
            this.btnYouTurnB.Text = "B";
            this.btnYouTurnB.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnYouTurnB.UseVisualStyleBackColor = false;
            this.btnYouTurnB.Click += new System.EventHandler(this.btnYouTurnB_Click);
            // 
            // btnYouTurnA
            // 
            this.btnYouTurnA.BackColor = System.Drawing.Color.Lime;
            this.btnYouTurnA.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.btnYouTurnA.Cursor = System.Windows.Forms.Cursors.Default;
            this.btnYouTurnA.Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnYouTurnA.Image = global::AgOpenGPS.Properties.Resources.ArrowLeft;
            this.btnYouTurnA.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnYouTurnA.Location = new System.Drawing.Point(32, 54);
            this.btnYouTurnA.Margin = new System.Windows.Forms.Padding(2);
            this.btnYouTurnA.Name = "btnYouTurnA";
            this.btnYouTurnA.Size = new System.Drawing.Size(68, 64);
            this.btnYouTurnA.TabIndex = 552;
            this.btnYouTurnA.Text = "A";
            this.btnYouTurnA.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnYouTurnA.UseVisualStyleBackColor = false;
            this.btnYouTurnA.Click += new System.EventHandler(this.btnYouTurnA_Click);
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
            this.btnExit.Location = new System.Drawing.Point(105, 54);
            this.btnExit.Name = "btnExit";
            this.btnExit.Size = new System.Drawing.Size(78, 64);
            this.btnExit.TabIndex = 211;
            this.btnExit.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnExit.UseVisualStyleBackColor = true;
            this.btnExit.Click += new System.EventHandler(this.btnExit_Click);
            // 
            // FormYouTurn
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.White;
            this.ClientSize = new System.Drawing.Size(278, 357);
            this.ControlBox = false;
            this.Controls.Add(this.btnturncircleA);
            this.Controls.Add(this.btnturncircleB);
            this.Controls.Add(this.btnFormYouTurnAch);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.btnYouTurnB);
            this.Controls.Add(this.btnYouTurnA);
            this.Controls.Add(this.cboxpRowWidthB);
            this.Controls.Add(this.cboxpRowWidthA);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.btnExit);
            this.Cursor = System.Windows.Forms.Cursors.Default;
            this.MaximizeBox = false;
            this.Name = "FormYouTurn";
            this.Text = "FormYouTurn";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnExit;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Button btnYouTurnA;
        private System.Windows.Forms.Button btnYouTurnB;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.ComboBox cboxpRowWidthA;
        private System.Windows.Forms.ComboBox cboxpRowWidthB;
        private System.Windows.Forms.Button btnFormYouTurnAch;
        private System.Windows.Forms.Button btnturncircleB;
        private System.Windows.Forms.Button btnturncircleA;
    }
}