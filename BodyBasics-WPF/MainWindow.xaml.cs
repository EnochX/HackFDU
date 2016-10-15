//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        /// <summary>
        /// Parameters define
        /// </summary>
        const int ACTION_1 = 1;
        const int ACTION_2 = 2;
        const int ACTION_3 = 3;
        const int STAT_INITIAL = 0;
        const int STAT_MOVING = 1;
        const int STAT_FINISH = 2;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        private const double StretchThreshold = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// Detect a truth action 
        /// </summary>
        private bool choose_flag_right;

        private bool choose_flag_left;

        /// <summary>
        /// Number of items selected
        /// </summary>
        private int numSelected = 0;

        /// <summary>
        /// No. of selected items
        /// </summary>
        private List<int> itemSelected = new List<int>();

        /// <summary>
        /// Status of the whole app
        /// </summary>
        private int app_status = 0;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        private int stretch_count;

        private bool in_Stretch;

        private int stretch_state;

        private double showtest;

        private bool show_color;

        private ColorFrameReader colorFrameReader = null;

        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
           
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            this.stretch_count = 0;

            this.in_Stretch = false;

            this.show_color = false;
            
            this.stretch_state = 0;

            this.choose_flag_right = false;

            this.choose_flag_left = false;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            // Set ui for initial status
            this.SetStatus(STAT_INITIAL);
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                if (show_color)
                {
                    return this.colorBitmap;
                }
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        public int StretchCount
        {
            get
            {
                return this.stretch_count;
            }

            set
            {
                if (this.stretch_count != value)
                {
                    this.stretch_count = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StretchCount"));
                    }
                }
            }
        }

        public double ShowTest
        {
            get
            {
                return this.showtest;
            }

            set
            {
                if (this.showtest != value)
                {
                    this.showtest = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("ShowTest"));
                    }
                }
            }
        }


        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            //this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            if(app_status == STAT_INITIAL)
                            {
                                this.ChooseItem(body.HandRightState, jointPoints[JointType.HandRight],body.HandLeftState,jointPoints[JointType.HandLeft]);
                            }

                            if(app_status == STAT_MOVING)
                            {
                                this.Stretch_Count(joints, jointPoints, 10);
                            }


                            
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void Stretch_Count(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, int max_count)
        {
            Joint joint_shoulder_right = joints[JointType.ShoulderRight];
            Joint joint_elbow_right = joints[JointType.ElbowRight];
            Joint joint_wrist_right = joints[JointType.WristRight];
            Joint joint_shoulder_left = joints[JointType.ShoulderLeft];
            Joint joint_elbow_left = joints[JointType.ElbowLeft];
            Joint joint_wrist_left = joints[JointType.WristLeft];



            if(this.stretch_state == 0)      //放下状态
            {
                if ((joint_elbow_right.Position.Y - joint_shoulder_right.Position.Y) * displayHeight >= -StretchThreshold && joint_wrist_right.Position.Y >= joint_shoulder_right.Position.Y)
                {
                    if ((joint_elbow_left.Position.Y - joint_shoulder_left.Position.Y) * displayHeight >= -StretchThreshold && joint_wrist_left.Position.Y >= joint_shoulder_left.Position.Y)
                    {
                        this.stretch_state = 1;
                        this.StretchCount++;
                        
                        if(stretch_count == max_count)
                        {
                            //完成！！！！！
                            this.stretch_count = 0;
                            this.in_Stretch = false;
                            this.app_status = 0;
                            this.SetStatus(STAT_FINISH);
                            return;
                        }
                    }
                }
            }

            else if(this.stretch_state == 1)    //举起状态
            {
                if (System.Math.Abs(joint_shoulder_right.Position.X - joint_wrist_right.Position.X) * displayWidth <= 3 * StretchThreshold && System.Math.Abs(joint_shoulder_right.Position.X - joint_elbow_right.Position.X) * displayWidth <= 3 * StretchThreshold && joint_shoulder_right.Position.Y > joint_wrist_right.Position.Y)
                {
                    if (System.Math.Abs(joint_shoulder_left.Position.X - joint_wrist_left.Position.X) * displayWidth <= 3 * StretchThreshold && System.Math.Abs(joint_shoulder_left.Position.X - joint_elbow_left.Position.X) * displayWidth <= 3 * StretchThreshold && joint_shoulder_left.Position.Y > joint_wrist_left.Position.Y)
                    {
                        this.stretch_state = 0;
                    }
                }
            }
        }


        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                }
            }
        }

        private void toAccount_Click(object sender, RoutedEventArgs e)
        {
        //    this.Content = new Account();
        }

        private void Action1_Click(object sender, RoutedEventArgs e)
        {
            Button btn = new Button()
            {
                Width = 160,
                Height = 50,
            };
            btn.Background = new ImageBrush
            {
                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action1.png"))
            };
            Canvas.SetTop(btn, numSelected * 52);
            Selected.Children.Add(btn);
            numSelected++;
            itemSelected.Add(ACTION_1);
        }

        private void Action2_Click(object sender, RoutedEventArgs e)
        {
            Button btn = new Button()
            {
                Width = 160,
                Height = 50,
            };
            btn.Background = new ImageBrush
            {
                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action2.png"))
            };
            Canvas.SetTop(btn, numSelected * 52);
            Selected.Children.Add(btn);
            numSelected++;
            itemSelected.Add(ACTION_2);
        }

        private void Action3_Click(object sender, RoutedEventArgs e)
        {
            Button btn = new Button()
            {
                Width = 160,
                Height = 50,
            };
            btn.Background = new ImageBrush
            {
                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action3.png"))
            };
            Canvas.SetTop(btn, numSelected * 52);
            Selected.Children.Add(btn);
            numSelected++;
            itemSelected.Add(ACTION_3);
        }

        private void CancelBtn_Click(object sender, RoutedEventArgs e)
        {
            if (numSelected > 0)
            {
                Selected.Children.RemoveAt(--numSelected);
                itemSelected.RemoveAt(itemSelected.Count - 1);
            }
        }

        private void StartBtn_Click(object sender, RoutedEventArgs e)
        {
            SetStatus(STAT_MOVING);
        }

        private void SetStatus(int stat)
        {
            app_status = stat;
            if (stat == STAT_INITIAL)//初始界面
            {
                BtnGroup.Visibility = Visibility.Visible;
                SelectItem.Visibility = Visibility.Visible;
                CounterBox.Visibility = Visibility.Hidden;
            }
            else if (stat == STAT_MOVING)//开始训练
            {
                BtnGroup.Visibility = Visibility.Hidden;
                SelectItem.Visibility = Visibility.Hidden;
                CounterBox.Visibility = Visibility.Visible;
            }
            else if (stat == STAT_FINISH)
            {
                BtnGroup.Visibility = Visibility.Visible;
                SelectItem.Visibility = Visibility.Visible;
                CounterBox.Visibility = Visibility.Hidden;
            }
        }

        private void ChooseItem(HandState handState1, Point handPosition1, HandState handState2, Point handPosition2)
        {
            if(handPosition1.Y>=80 && handPosition1.Y<=180)
            {
                if (handPosition1.X >= 25 && handPosition1.X <= 150)
                {
                    if (handState1 == HandState.Closed)
                    {
                        choose_flag_right = true;
                    }
                    else if (handState1 == HandState.Open)
                    {
                        if (choose_flag_right)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action1.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_1);
                            choose_flag_right = false;
                        }
                    }
                }
                else if (handPosition1.X >= 200 && handPosition1.X <= 300)
                {
                    if (handState1 == HandState.Closed)
                    {
                        choose_flag_right = true;
                    }
                    else if (handState1 == HandState.Open)
                    {
                        if (choose_flag_right)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action2.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_2);
                            choose_flag_right = false;
                        }
                    }
                }
                else if (handPosition1.X >= 380 && handPosition1.X <= 450)
                {
                    if (handState1 == HandState.Closed)
                    {
                        choose_flag_right = true;
                    }
                    else if (handState1 == HandState.Open)
                    {
                        if (choose_flag_right)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action3.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_3);
                            choose_flag_right = false;
                        }
                    }
                }
            }


            else if(handPosition1.Y>=240 && handPosition1.Y<=300)
            {
                if(handPosition1.X>=0 && handPosition1.X<=200)
                {
                    if (handState1 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState1 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            if (numSelected > 0)
                            {
                                Selected.Children.RemoveAt(--numSelected);
                                itemSelected.RemoveAt(itemSelected.Count - 1);
                            }
                            choose_flag_left = false;
                        }
                    }
                }
                else if (handPosition1.X >= 220 && handPosition1.X <= 480)
                {
                    if (handState1 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState1 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            SetStatus(STAT_MOVING);
                            choose_flag_left = false;
                        }
                    }
                }
            }

            if (handPosition2.Y >= 80 && handPosition2.Y <= 180)
            {
                if (handPosition2.X >= 25 && handPosition2.X <= 150)
                {
                    if (handState2 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState2 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action1.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_1);
                            choose_flag_left = false;
                        }
                    }
                }
                else if (handPosition2.X >= 200 && handPosition2.X <= 300)
                {
                    if (handState2 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState2 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action2.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_2);
                            choose_flag_left = false;
                        }
                    }
                }
                else if (handPosition2.X >= 380 && handPosition2.X <= 450)
                {
                    if (handState2 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState2 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            Button btn = new Button()
                            {
                                Width = 160,
                                Height = 50,
                            };
                            btn.Background = new ImageBrush
                            {
                                ImageSource = new BitmapImage(new Uri("pack://application:,,,/Images/action3.png"))
                            };
                            Canvas.SetTop(btn, numSelected * 52);
                            Selected.Children.Add(btn);
                            numSelected++;
                            itemSelected.Add(ACTION_3);
                            choose_flag_left = false;
                        }
                    }
                }
            }


            else if (handPosition2.Y >= 240 && handPosition2.Y <= 300)
            {
                if (handPosition2.X >= 0 && handPosition2.X <= 200)
                {
                    if (handState2 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState2 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            if (numSelected > 0)
                            {
                                Selected.Children.RemoveAt(--numSelected);
                                itemSelected.RemoveAt(itemSelected.Count - 1);
                            }
                            choose_flag_left = false;
                        }
                    }
                }
                else if (handPosition2.X >= 220 && handPosition2.X <= 480)
                {
                    if (handState2 == HandState.Closed)
                    {
                        choose_flag_left = true;
                    }
                    else if (handState2 == HandState.Open)
                    {
                        if (choose_flag_left)
                        {
                            SetStatus(STAT_MOVING);
                            choose_flag_left = false;
                        }
                    }
                }

            }




        }
    }
}
