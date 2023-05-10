namespace AdaptiveAlgorithmCode.src
{
    public class MeasurementConfig
    {
        public string MeasurementName;
        public Dictionary<ushort, Position> Positions;
        public Quadruple[] Quadruples;
    }

    public class Position
    {
        public double X;
        public double Y;
        public double Z;

        public Position( double x, double y, double z )
        {
            X = x;
            Y = y;
            Z = z;
        }
    }
}
