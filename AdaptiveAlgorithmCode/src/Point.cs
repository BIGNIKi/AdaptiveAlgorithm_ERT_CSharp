using DelaunatorSharp;
using System.Numerics;

namespace AdaptiveAlgorithmCode.src
{
    public class Point : IPoint
    {
        private Vector2 _vector2;

        public Point( Vector2 vec )
        {
            _vector2 = vec;
        }

        public double X { get => _vector2.X; set => _vector2.X = (float)value; }
        public double Y { get => _vector2.Y; set => _vector2.Y = (float)value; }
    }
}
