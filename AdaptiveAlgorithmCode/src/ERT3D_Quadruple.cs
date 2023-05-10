using MathNet.Spatial.Euclidean;

namespace AdaptiveAlgorithmCode.src
{
    public class ERT3D_Quadruple
    {
        private int? indexRemote_1;
        private int? indexRemote_2;
        private int indexA;
        private int indexB;
        private int indexM;
        private int indexN;

        private Point2D coordA;
        private Point2D coordB;
        private Point2D coordM;
        private Point2D coordN;

        private double lenAB;
        private double lenMN;
        private double lenAM;
        private double lenBM;
        private double lenAN;
        private double lenBN;
        private double largestLen;
        private double angleABMN;
        private double geometryFactor;
        private double geometryFactorInverseNoPi;
        private double geometryFactorStraightNoPi;
        private double measurementPointDepth;
        private bool geometryFactorIsPositive;

        private Point2D midpointAB;
        private Point2D midpointMN;

        private Point3D measurementPointXYZ;
        public int? IndexRemote_1 { get => indexRemote_1; set => indexRemote_1 = value; }
        public int? IndexRemote_2 { get => indexRemote_2; set => indexRemote_2 = value; }
        public int IndexA { get => indexA; set => indexA = value; }
        public int IndexB { get => indexB; set => indexB = value; }
        public int IndexM { get => indexM; set => indexM = value; }
        public int IndexN { get => indexN; set => indexN = value; }
        public Point2D CoordA { get => coordA; set => coordA = value; }
        public Point2D CoordB { get => coordB; set => coordB = value; }
        public Point2D CoordM { get => coordM; set => coordM = value; }
        public Point2D CoordN { get => coordN; set => coordN = value; }
        //public Point2D MidpointAB { get => midpointAB; }
        //public Point2D MidpointMN { get => midpointMN; }
        public double LenAB { get => lenAB; }
        public double LenMN { get => lenMN; }
        public double LenAM { get => lenAM; }
        public double LenBM { get => lenBM; }
        public double LenAN { get => lenAN; }
        public double LenBN { get => lenBN; }
        public Point3D MeasurementPointXYZ { get => measurementPointXYZ; set => measurementPointXYZ = value; }
        public double AngleABMN { get => angleABMN; }
        public double GeometryFactor_K { get => geometryFactor; }
        public double GeometryFactorStraightNoPi { get => geometryFactorStraightNoPi; }
        public bool GeometryFactorIsPositive { get => geometryFactorIsPositive; }

        public ERT3D_Quadruple( int indexA, int indexB, int indexM, int indexN, int? indexRemote_1 = null, int? indexRemote_2 = null )
        {
            IndexRemote_1 = indexRemote_1;
            IndexRemote_2 = indexRemote_2;
            IndexA = indexA;
            IndexB = indexB;
            IndexM = indexM;
            IndexN = indexN;
            lenAB = 0;
            lenMN = 0;
            lenAM = 0;
            lenBM = 0;
            lenAN = 0;
            lenBN = 0;
            angleABMN = 0;
            geometryFactor = Double.PositiveInfinity;
            measurementPointDepth = 0;
            geometryFactorIsPositive = true;
            IndexRemote_1 = indexRemote_1;
            IndexRemote_2 = indexRemote_2;
        }
        public ERT3D_Quadruple( int indexA, int indexB, int indexM, int indexN,
            Point2D coordA, Point2D coordB, Point2D coordM, Point2D coordN, int? indexRemote_1 = null, int? indexRemote_2 = null )
        {
            IndexRemote_1 = indexRemote_1;
            IndexRemote_2 = indexRemote_2;
            IndexA = indexA;
            IndexB = indexB;
            IndexM = indexM;
            IndexN = indexN;
            CoordA = coordA;
            CoordB = coordB;
            CoordM = coordM;
            CoordN = coordN;
            if(CoordA == CoordB || CoordA == CoordM || CoordA == CoordN || CoordB == CoordM ||
                CoordB == CoordN || CoordM == CoordN)
            {
                throw new Exception("Coordinates of electrodes should not be equal!");
            }
            //midpointAB = Point2D.MidPoint(coordA, coordB);
            //midpointMN = Point2D.MidPoint(coordM, coordN);
            Line2D lineAB = new Line2D(coordA, coordB);
            Line2D lineMN = new Line2D(coordM, coordN);
            lenAB = lineAB.Length;
            lenMN = lineMN.Length;

            Line2D lineAM = new Line2D(coordA, coordM);
            Line2D lineBM = new Line2D(coordB, coordM);
            Line2D lineAN = new Line2D(coordA, coordN);
            Line2D lineBN = new Line2D(coordB, coordN);
            lenAM = lineAM.Length;
            lenBM = lineBM.Length;
            lenAN = lineAN.Length;
            lenBN = lineBN.Length;
            angleABMN = CalculateAngleBetweenAB(lineAB, lineMN);

            //if (IndexRemote_1 != null && IndexRemote_2 != null)
            //{
            //    if (IndexRemote_1 == IndexA)
            //    {
            //        lenAM = double.PositiveInfinity;
            //    }
            //    else //this is IndexB
            //    {
            //        lenBM = double.PositiveInfinity;
            //    }
            //    if (IndexRemote_2 == IndexN)
            //    {
            //        lenAN = double.PositiveInfinity;
            //    }
            //    else //this is IndexM
            //    {
            //        lenAM = double.PositiveInfinity;
            //    }
            //}
            //else if (IndexRemote_1 != null)
            //{
            //    if (IndexRemote_1 == IndexA)
            //    {
            //        lenAM = double.PositiveInfinity;
            //    }
            //    else //this is IndexB
            //    {
            //        lenBM = double.PositiveInfinity;
            //    }
            //}
            //else if (IndexRemote_2 != null)
            //{
            //    if (IndexRemote_2 == IndexN)
            //    {
            //        lenAN = double.PositiveInfinity;
            //    }
            //    else //this is IndexM
            //    {
            //        lenAM = double.PositiveInfinity;
            //    }
            //}            
            geometryFactorInverseNoPi = 1 / lenAM - 1 / lenBM - 1 / lenAN + 1 / lenBN;

            geometryFactorIsPositive = geometryFactorInverseNoPi > 0;
            if(Math.Abs(geometryFactorInverseNoPi) < 1e-6)
            {
                geometryFactorStraightNoPi = geometryFactorIsPositive ? double.PositiveInfinity : double.NegativeInfinity;
                geometryFactor = geometryFactorIsPositive ? double.PositiveInfinity : double.NegativeInfinity;
            }
            else
            {
                geometryFactorStraightNoPi = 1 / geometryFactorInverseNoPi;
                geometryFactor = 2 * Math.PI / geometryFactorInverseNoPi;
            }
            (double xPos, double yPos) = CalculateXYPosForMeasurementPointByLargestDistance();

            measurementPointXYZ = new Point3D(xPos, yPos, -largestLen / 5);
        }
        public double CalculateMeasurementPointDepth( double lBN, double lBM, double lAN, double lAM )
        {
            const double initialDepth = 0.1;
            const double initialDepthQ = 0.04;
            double depth = initialDepth;
            double depthQ = initialDepthQ;
            double sensitivity = 0;
            while(sensitivity < 0.95)
            {
                double someConstant = 1 / Math.Sqrt(lBN + depthQ) - 1 / Math.Sqrt(lBM + depthQ) - 1 / Math.Sqrt(lAN + depthQ)
                    + 1 / Math.Sqrt(lAM + depthQ);
                sensitivity = 1 - someConstant / geometryFactorInverseNoPi;
                depth *= 1.1;
                depthQ = 4 * depth * depth;
            }
            return depth;
        }
        private (double X, double Y) CalculateXYPosForMeasurementPointByLargestDistance()
        {
            double fAM = Math.Pow(lenAM, 2);
            double fAN = Math.Pow(lenAN, 2);
            double fBM = Math.Pow(lenBM, 2);
            double fBN = Math.Pow(lenBN, 2);
            double fAB = Math.Pow(lenAB, 2);
            double fMN = Math.Pow(lenMN, 2);
            double xPos = 0;
            double yPos = 0;


            if(IndexRemote_1 != null && IndexRemote_2 != null)
            {
                if(IndexRemote_1 == IndexB && IndexRemote_2 == IndexN)
                {
                    // A + M
                    xPos = (CoordA.X + CoordM.X) / 2.0;
                    yPos = (CoordA.Y + CoordM.Y) / 2.0;
                    largestLen = lenAM;
                    return (xPos, yPos);
                }
                if(IndexRemote_1 == IndexA && IndexRemote_2 == IndexN)
                {
                    // B + M
                    xPos = (CoordB.X + CoordM.X) / 2.0;
                    yPos = (CoordB.Y + CoordM.Y) / 2.0;
                    largestLen = lenBM;
                    return (xPos, yPos);
                }
                if(IndexRemote_1 == IndexB && IndexRemote_2 == IndexM)
                {
                    // A + N
                    xPos = (CoordA.X + CoordN.X) / 2.0;
                    yPos = (CoordA.Y + CoordN.Y) / 2.0;
                    largestLen = lenAN;
                    return (xPos, yPos);
                }
                if(IndexRemote_1 == IndexA && IndexRemote_2 == IndexM)
                {
                    // B + N
                    xPos = (CoordB.X + CoordN.X) / 2.0;
                    yPos = (CoordB.Y + CoordN.Y) / 2.0;
                    largestLen = lenBN;
                    return (xPos, yPos);
                }
            }
            else if(IndexRemote_1 == IndexA || IndexRemote_1 == IndexB)
            {
                // M + N
                xPos = (CoordM.X + CoordN.X) / 2.0;
                yPos = (CoordM.Y + CoordN.Y) / 2.0;
                largestLen = lenMN;
                return (xPos, yPos);
            }
            else if(IndexRemote_2 == IndexM || IndexRemote_2 == IndexN)
            {
                // A + B
                xPos = (CoordA.X + CoordB.X) / 2.0;
                yPos = (CoordA.Y + CoordB.Y) / 2.0;
                largestLen = lenAB;
                return (xPos, yPos);
            }
            if((fAM >= fAN) && (fAM >= fBM) && (fAM >= fBN) && (fAM >= fAB) && (fAM >= fMN))
            {
                // A + M
                xPos = (CoordA.X + CoordM.X) / 2.0;
                yPos = (CoordA.Y + CoordM.Y) / 2.0;
                largestLen = lenAM;
            }
            else
            {
                if((fAN >= fAM) && (fAN >= fBM) && (fAN >= fBN) && (fAN >= fAB) && (fAN >= fMN))
                {
                    // A + N
                    xPos = (CoordA.X + CoordN.X) / 2.0;
                    yPos = (CoordA.Y + CoordN.Y) / 2.0;
                    largestLen = lenAN;
                }
                else
                {
                    if((fBN >= fAM) && (fBN >= fBM) && (fBN >= fAN) && (fBN >= fAB) && (fBN >= fMN))
                    {
                        // B + N
                        xPos = (CoordB.X + CoordN.X) / 2.0;
                        yPos = (CoordB.Y + CoordN.Y) / 2.0;
                        largestLen = lenBN;
                    }
                    else
                    {
                        if((fBM >= fAM) && (fBM >= fBN) && (fBM >= fAN) && (fBM >= fAB) && (fBM >= fMN))
                        {
                            // B + M
                            xPos = (CoordB.X + CoordM.X) / 2.0;
                            yPos = (CoordB.Y + CoordM.Y) / 2.0;
                            largestLen = lenBM;
                        }
                        else
                        {
                            if((fMN >= fAM) && (fMN >= fBN) && (fMN >= fAN) && (fMN >= fAB) && (fMN >= fBM))
                            {
                                // M + N
                                xPos = (CoordM.X + CoordN.X) / 2.0;
                                yPos = (CoordM.Y + CoordN.Y) / 2.0;
                                largestLen = lenMN;
                            }
                            else
                            {
                                // A + B
                                xPos = (CoordA.X + CoordB.X) / 2.0;
                                yPos = (CoordA.Y + CoordB.Y) / 2.0;
                                largestLen = lenAB;
                            }
                        }
                    }
                }
            }
            return (xPos, yPos);
        }
        private double CalculateAngleBetweenAB( Line2D lineAB, Line2D lineMN )
        {
            double angle = 0;
            //shift lines to (0,0)
            double shiftX = 0 - lineAB.StartPoint.X;
            double shiftY = 0 - lineAB.StartPoint.Y;
            Vector2D vectorAB = new Vector2D(lineAB.EndPoint.X + shiftX, lineAB.EndPoint.Y + shiftY);
            shiftX = 0 - lineMN.StartPoint.X;
            shiftY = 0 - lineMN.StartPoint.Y;
            Vector2D vectorMN = new Vector2D(lineMN.EndPoint.X + shiftX, lineMN.EndPoint.Y + shiftY);
            angle = vectorAB.AngleTo(vectorMN).Degrees <= 90 ? vectorAB.AngleTo(vectorMN).Degrees : 180 - vectorAB.AngleTo(vectorMN).Degrees;
            return angle;
        }

    }
}
