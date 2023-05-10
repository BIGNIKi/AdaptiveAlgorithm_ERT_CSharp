namespace AdaptiveAlgorithmCode.src
{
    public class ERT3D_ComputeMeasurementPointPos
    {
        public static void ComputeMeasurementPointPosMultipleConcurrentNoLog( List<ERT3D_Quadruple> quadruplesToCompute, double step )
        {
            int curnum = 0;
            int qTotal = quadruplesToCompute.Count;

            Parallel.ForEach
                (
                    quadruplesToCompute, item =>
                    {
                        item.MeasurementPointXYZ = new MathNet.Spatial.Euclidean.Point3D(
                            item.MeasurementPointXYZ.X, item.MeasurementPointXYZ.Y,
                            ComputeMeasurementPointDepth(item, step));
                        Interlocked.Add(ref curnum, 1);
                    }
            );
        }

        // Расчет кажущейся глубины для квадруполи
        public static double ComputeMeasurementPointDepth( ERT3D_Quadruple quadrupleToCompute, double step )
        {
            double zPosNaive = quadrupleToCompute.MeasurementPointXYZ.Z;
            double zPos = 0;
            double fAM = Math.Pow(quadrupleToCompute.LenAM, 2);
            double fAN = Math.Pow(quadrupleToCompute.LenAN, 2);
            double fBM = Math.Pow(quadrupleToCompute.LenBM, 2);
            double fBN = Math.Pow(quadrupleToCompute.LenBN, 2);
            double fAB = Math.Pow(quadrupleToCompute.LenAB, 2);
            double fMN = Math.Pow(quadrupleToCompute.LenMN, 2);
            double GFactor = quadrupleToCompute.GeometryFactorStraightNoPi;
            //double GFactor = 1;

            Func<double, double> fImpl = delegate (double x)
            {
                return 4.0 * x * GFactor * (Math.Pow((fAM + 4 * x * x), -1.5) - Math.Pow((fAN + 4 * x * x), -1.5) - Math.Pow((fBM + 4 * x * x), -1.5) + Math.Pow((fBN + 4 * x * x), -1.5));
            };

            double increment = step / 10;
            zPos = increment;
            int iter = 0;
            while(true)
            {
                if(iter >= 1000)
                {
                    break;
                }
                double res = MathNet.Numerics.Integration.NewtonCotesTrapeziumRule.IntegrateAdaptive(fImpl, 0.0, zPos, increment / 100);
                if(res >= 0.5)
                {
                    break;
                }
                if(double.IsNaN(res)) // integration failed, returning "naive" zPos
                {
                    return zPosNaive;
                }
                zPos += increment;
                iter++;
            }
            //quadrupleToCompute.MeasurementPointXYZ = new MathNet.Spatial.Euclidean.Point3D(quadrupleToCompute.MeasurementPointXYZ.X,
            //	quadrupleToCompute.MeasurementPointXYZ.Y, -zPos);		

            return -zPos;
        }
    }
}
