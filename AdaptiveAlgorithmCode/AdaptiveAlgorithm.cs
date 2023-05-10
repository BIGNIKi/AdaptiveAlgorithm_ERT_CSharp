using AdaptiveAlgorithmCode.src;
using DelaunatorSharp;
using System.Data;
using System.Numerics;
using System.Xml.Serialization;
using System.Xml;
using MathNet.Spatial.Euclidean;

namespace AdaptiveAlgorithmCode
{
    public class AdaptiveAlgorithm
    {
        private readonly Options _options;
        private readonly MeasurementConfig[] _measurementConfigs;
        private readonly IToolERT _toolERT;
        private readonly float _initalPercentPoints;
        private readonly float _resistanceValueToSeal;

        public AdaptiveAlgorithm(Options options, MeasurementConfig[] measurementConfigs, IToolERT s, float InitalPercentPoints, float ResistanceValueToSeal)
        {
            _options = options;
            _measurementConfigs = measurementConfigs;
            _toolERT = s;
            _initalPercentPoints = InitalPercentPoints;
            _resistanceValueToSeal = ResistanceValueToSeal;
        }

        public void Run()
        {
            var coordsArray = GetDataWithPointsCoordinates();

            var pointsForCurrentIteration = new List<(Vector2, Quadruple, QuadMeasurement)>(); // все данные на текущую итерацию

            //TakeEquidistantPoints(pointsForCurrentIteration, coordsArray);
            //FindPointsWithOptimalArea(pointsForCurrentIteration, coordsArray);
            FindInitialPoints(pointsForCurrentIteration, coordsArray);

            var timeStart = DateTime.UtcNow; // важно учесть, что длительность изначального подбора точек (FindPointsWithOptimalArea) сейчас не учитывается

            int numOfIteration = 1;

            while (true)
            {
                Console.WriteLine($"\nStarted {numOfIteration} iteration.\n");

                StartMeasurement(pointsForCurrentIteration, numOfIteration);

                WritePointsToFile(pointsForCurrentIteration); // вызывать для отладки и визуализации в проекте ViewERT.WPF_tests

                var deloneGraph = new Delaunator(MakePointsArrayForAlgo(pointsForCurrentIteration));

                var resistanceToSeal = CalculateResistanceToSeal(pointsForCurrentIteration);

                var newPoints = FormNewPointsForNewIteration(deloneGraph, coordsArray, pointsForCurrentIteration, resistanceToSeal); // подбираем новые точки для уплотнения
                if (newPoints.Count == 0)
                {
                    ConsoleLogTimeOnIteration(timeStart, numOfIteration);
                    break;
                }
                foreach (var item in newPoints) // добавляем в лист, который пойдет на измерения
                {
                    pointsForCurrentIteration.Add((item.Item1, item.Item2, new QuadMeasurement(-1, -1, -1, -1, -1, -1)));
                }

                ConsoleLogTimeOnIteration(timeStart, numOfIteration);

                numOfIteration++;
            }
        }

        private float CalculateResistanceToSeal(List<(Vector2, Quadruple, QuadMeasurement)> pointsForCurrentIteration)
        {
            float maxResistance = float.MinValue;
            float minResistance = float.MaxValue;
            foreach (var item in pointsForCurrentIteration)
            {
                if (maxResistance < item.Item3.Resistance)
                {
                    maxResistance = item.Item3.Resistance;
                }
                if (minResistance > item.Item3.Resistance)
                {
                    minResistance = item.Item3.Resistance;
                }
            }

            return (maxResistance - minResistance) * (100f - _resistanceValueToSeal) / 100f;
        }

        public static src.Point[] MakePointsArrayForAlgo(List<(Vector2, Quadruple, QuadMeasurement)> pointsForCurrentIteration)
        {
            var result = new src.Point[pointsForCurrentIteration.Count];
            for (int i = 0; i < pointsForCurrentIteration.Count; i++)
            {
                var @new = new src.Point(pointsForCurrentIteration[i].Item1);
                result[i] = @new;
            }

            return result;
        }

        private static void ConsoleLogTimeOnIteration(DateTime timeStart, int numOfIteration)
        {
            var dif = DateTime.UtcNow - timeStart;
            if (dif.TotalMinutes >= 1)
            {
                Console.WriteLine($"{numOfIteration} iteration took {dif.TotalMinutes} minutes.");
            }
            else if (dif.TotalSeconds >= 1)
            {
                Console.WriteLine($"{numOfIteration} iteration took {dif.TotalSeconds} seconds.");
            }
            else
            {
                Console.WriteLine($"{numOfIteration} iteration took {dif.TotalMilliseconds} milliseconds.");
            }
        }

        private List<(Vector2, Quadruple)> FormNewPointsForNewIteration(Delaunator graph,
            List<(Vector2, Quadruple)> remainingPoints,
            List<(Vector2, Quadruple, QuadMeasurement)> pointsForCurrentIteration,
            float resistanceValueToSeal)
        {
            var newPointsForIteration = new List<(Vector2, Quadruple)>();

            // перебор всех рёбер
            for (int e = 0; e < graph.Triangles.Length; e++)
            {
                if (e > graph.Halfedges[e])
                {
                    var p = graph.Points[graph.Triangles[e]];
                    var q = graph.Points[graph.Triangles[GetNextHalfEdge(e)]];

                    (Vector2, Quadruple, QuadMeasurement) resistanceP = pointsForCurrentIteration.Where(x => x.Item1.X == p.X && x.Item1.Y == p.Y).First();
                    (Vector2, Quadruple, QuadMeasurement) resistanceQ = pointsForCurrentIteration.Where(x => x.Item1.X == q.X && x.Item1.Y == q.Y).First();

                    // если выполнено условие, нужно уплотнить граф
                    var difference = Math.Abs(resistanceP.Item3.Resistance - resistanceQ.Item3.Resistance);
                    if (difference >= resistanceValueToSeal)
                    {
                        var idOfPoint = FindCentralPoint(resistanceP.Item1, resistanceQ.Item1, remainingPoints);
                        if (idOfPoint != -1)
                        {
                            newPointsForIteration.Add(remainingPoints[idOfPoint]);
                            remainingPoints.RemoveAt(idOfPoint);
                        }
                    }
                }
            }

            return newPointsForIteration;
        }

        public static int GetNextHalfEdge(int e)
        {
            return e % 3 == 2 ? e - 2 : e + 1;
        }

        /// <summary>
        /// Находит id точки в remainingPoints, которая будет максимально близко к центру между точками a и b.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="remainingPoints"></param>
        private int FindCentralPoint(Vector2 a, Vector2 b, List<(Vector2, Quadruple)> remainingPoints)
        {
            if (a.X > b.X)
            {
                var temp = b.X;
                b.X = a.X;
                a.X = temp;
            }
            if (a.Y > b.Y)
            {
                var temp = b.Y;
                b.Y = a.Y;
                a.Y = temp;
            }

            var pointsInside = new List<int>();
            for (int i = 0; i < remainingPoints.Count; i++)
            {
                if (remainingPoints[i].Item1.X >= a.X && b.X >= remainingPoints[i].Item1.X && remainingPoints[i].Item1.Y >= a.Y && b.Y >= remainingPoints[i].Item1.Y)
                {
                    pointsInside.Add(i);
                }
            }

            var center = new Vector2(b.X - (b.X - a.X) / 2, b.Y - (b.Y - a.Y) / 2); // центр между точками a и b

            var min = float.MaxValue;
            int idWithMin = -1;
            for (int i = 0; i < pointsInside.Count; i++)
            {
                var distance = Vector2.Distance(center, remainingPoints[pointsInside[i]].Item1);

                if (min > distance)
                {
                    min = distance;
                    idWithMin = pointsInside[i];
                }
            }

            return idWithMin;
        }

        /// <summary>
        /// Решение в тупую. Из allPoints набираются равноотстоящие точки и кладутся в finalArrayWithInitialPoints. 
        /// </summary>
        /// <param name="finalArrayWithInitialPoints"></param>
        /// <param name="allPoints"></param>
        private void TakeEquidistantPoints(List<(Vector2, Quadruple, QuadMeasurement)> finalArrayWithInitialPoints, List<(Vector2, Quadruple)> allPoints)
        {
            var selectedPoints = new List<int>();

            var xMin = float.MaxValue;
            var idXMin = -1;
            var xMax = float.MinValue;
            var idXMax = -1;
            var yMin = float.MaxValue;
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.X < xMin)
                {
                    xMin = allPoints[i].Item1.X;
                    idXMin = i;
                }
                if (allPoints[i].Item1.X > xMax)
                {
                    xMax = allPoints[i].Item1.X;
                    idXMax = i;
                }
                if (allPoints[i].Item1.Y < yMin)
                {
                    yMin = allPoints[i].Item1.Y;
                }
            }
            selectedPoints.Add(idXMin);
            selectedPoints.Add(idXMax);
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.Y == yMin)
                {
                    selectedPoints.Add(i);
                }
            }

            int numOfPoints = (int)(allPoints.Count * _initalPercentPoints / 100f);

            int everyNth = allPoints.Count / numOfPoints;

            for (int i = 0; i < allPoints.Count; i += everyNth)
            {
                if (selectedPoints.Contains(i))
                {
                    continue;
                }

                selectedPoints.Add(i);
            }

            for (int i = 0; i < selectedPoints.Count; i++)
            {
                var @new = (allPoints[selectedPoints[i]].Item1, allPoints[selectedPoints[i]].Item2, new QuadMeasurement(-1, -1, -1, -1, -1, -1));
                finalArrayWithInitialPoints.Add(@new);
            }

            selectedPoints.Sort();

            for (int i = selectedPoints.Count - 1; i >= 0; i--)
            {
                allPoints.RemoveAt(selectedPoints[i]);
            }
        }


        /// <summary>
        /// Подход, основанный на уплотнении треугольника в триангуляции Делоне, у которого максимальная площадь
        /// </summary>
        /// <param name="finalArrayWithInitialPoints"></param>
        /// <param name="allPoints"></param>
        private void FindInitialPoints(List<(Vector2, Quadruple, QuadMeasurement)> finalArrayWithInitialPoints, List<(Vector2, Quadruple)> allPoints)
        {
            var selectedPoints = new List<int>();

            var xMin = float.MaxValue;
            var idXMin = -1;
            var xMax = float.MinValue;
            var idXMax = -1;
            var yMin = float.MaxValue;
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.X < xMin)
                {
                    xMin = allPoints[i].Item1.X;
                    idXMin = i;
                }
                if (allPoints[i].Item1.X > xMax)
                {
                    xMax = allPoints[i].Item1.X;
                    idXMax = i;
                }
                if (allPoints[i].Item1.Y < yMin)
                {
                    yMin = allPoints[i].Item1.Y;
                }
            }
            selectedPoints.Add(idXMin);
            selectedPoints.Add(idXMax);
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.Y == yMin)
                {
                    selectedPoints.Add(i);
                    break;
                }
            }

            int numOfPoints = (int)(allPoints.Count * _initalPercentPoints / 100f);

            while (selectedPoints.Count < numOfPoints)
            {
                // create Delone graph
                var result = new src.Point[selectedPoints.Count];
                for (int i = 0; i < selectedPoints.Count; i++)
                {
                    var @new = new src.Point(allPoints[selectedPoints[i]].Item1);
                    result[i] = @new;
                }
                var deloneGraph = new Delaunator(result);

                int iMaxTriangleArea = -1; // id треугольника с максимальной площадью
                float maxArea = -1;
                // перебор всех треугольников с целью найти треугольник с наибольшей площадью
                for (int i = 0; i < deloneGraph.Triangles.Length / 3; i++)
                {
                    int firstId = deloneGraph.Triangles[3 * i];
                    int secondId = deloneGraph.Triangles[3 * i + 1];
                    int thirdId = deloneGraph.Triangles[3 * i + 2];

                    float lengthA = (float)Math.Sqrt(
                        Math.Pow(deloneGraph.Points[firstId].X - deloneGraph.Points[secondId].X, 2) +
                        Math.Pow(deloneGraph.Points[firstId].Y - deloneGraph.Points[secondId].Y, 2));
                    float lengthB = (float)Math.Sqrt(
                        Math.Pow(deloneGraph.Points[firstId].X - deloneGraph.Points[thirdId].X, 2) +
                        Math.Pow(deloneGraph.Points[firstId].Y - deloneGraph.Points[thirdId].Y, 2));
                    float lengthC = (float)Math.Sqrt(
                        Math.Pow(deloneGraph.Points[secondId].X - deloneGraph.Points[thirdId].X, 2) +
                        Math.Pow(deloneGraph.Points[secondId].Y - deloneGraph.Points[thirdId].Y, 2));
                    float polyP = (lengthA + lengthB + lengthC) / 2;
                    float area = (float)Math.Sqrt(polyP * (polyP - lengthA) * (polyP - lengthB) * (polyP - lengthC));
                    if (area > maxArea)
                    {
                        maxArea = area;
                        iMaxTriangleArea = i;
                    }
                }

                int firstP = deloneGraph.Triangles[3 * iMaxTriangleArea];
                int secondP = deloneGraph.Triangles[3 * iMaxTriangleArea + 1];
                int thirdP = deloneGraph.Triangles[3 * iMaxTriangleArea + 2];

                // нашли центр треугольника
                float centerX = (float)((deloneGraph.Points[firstP].X + deloneGraph.Points[secondP].X + deloneGraph.Points[thirdP].X) / 3);
                float centerY = (float)((deloneGraph.Points[firstP].Y + deloneGraph.Points[secondP].Y + deloneGraph.Points[thirdP].Y) / 3);
                Vector2 center = new(centerX, centerY);

                var minDist = float.MaxValue;
                int pointToAddId = -1;
                // find point which is maximal near with center of triangle
                for (int j = 0; j < allPoints.Count; j++)
                {
                    if (selectedPoints.Contains(j))
                    {
                        continue;
                    }

                    var currentPoint = new Vector2(allPoints[j].Item1.X, allPoints[j].Item1.Y);

                    if (!IsInsideTriangle(currentPoint,
                        new((float)deloneGraph.Points[firstP].X, (float)deloneGraph.Points[firstP].Y),
                        new((float)deloneGraph.Points[secondP].X, (float)deloneGraph.Points[secondP].Y),
                        new((float)deloneGraph.Points[thirdP].X, (float)deloneGraph.Points[thirdP].Y)))
                    {
                        continue;
                    }

                    var dist = Vector2.Distance(center, currentPoint);
                    if (minDist > dist)
                    {
                        minDist = dist;
                        pointToAddId = j;
                    }
                }

                if (pointToAddId != -1)
                {
                    selectedPoints.Add(pointToAddId);
                    if (selectedPoints.Count >= numOfPoints)
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }

            }

            for (int i = 0; i < selectedPoints.Count; i++)
            {
                var @new = (allPoints[selectedPoints[i]].Item1, allPoints[selectedPoints[i]].Item2, new QuadMeasurement(-1, -1, -1, -1, -1, -1));
                finalArrayWithInitialPoints.Add(@new);
            }

            selectedPoints.Sort();

            for (int i = selectedPoints.Count - 1; i >= 0; i--)
            {
                allPoints.RemoveAt(selectedPoints[i]);
            }
        }

        private static bool IsInsideTriangle(Vector2 point, Vector2 vertex1, Vector2 vertex2, Vector2 vertex3)
        {
            var p = VectMul(new(vertex1.X - point.X, vertex1.Y - point.Y), new(vertex2.X - vertex1.X, vertex2.Y - vertex1.Y));
            var q = VectMul(new(vertex2.X - point.X, vertex2.Y - point.Y), new(vertex3.X - vertex2.X, vertex3.Y - vertex2.Y));
            var r = VectMul(new(vertex3.X - point.X, vertex3.Y - point.Y), new(vertex1.X - vertex3.X, vertex1.Y - vertex3.Y));

            if (p <= 0 && q <= 0 && r <= 0 || p >= 0 && q >= 0 && r >= 0)
                return true;
            else
                return false;
        }

        private static float VectMul(Vector2 point1, Vector2 point2)
        {
            return point1.X * point2.Y - point1.Y * point2.X;
        }

        /// <summary>
        /// ОЧЕНЬ долгий, но точный метод выбора начальных точек измерения.
        /// </summary>
        /// <param name="finalArrayWithInitialPoints"></param>
        /// <param name="allPoints"></param>
        private void FindPointsWithOptimalArea(List<(Vector2, Quadruple, QuadMeasurement)> finalArrayWithInitialPoints, List<(Vector2, Quadruple)> allPoints)
        {
            var selectedPoints = new List<int>();

            var xMin = float.MaxValue;
            var idXMin = -1;
            var xMax = float.MinValue;
            var idXMax = -1;
            var yMin = float.MaxValue;
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.X < xMin)
                {
                    xMin = allPoints[i].Item1.X;
                    idXMin = i;
                }
                if (allPoints[i].Item1.X > xMax)
                {
                    xMax = allPoints[i].Item1.X;
                    idXMax = i;
                }
                if (allPoints[i].Item1.Y < yMin)
                {
                    yMin = allPoints[i].Item1.Y;
                }
            }
            selectedPoints.Add(idXMin);
            selectedPoints.Add(idXMax);
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].Item1.Y == yMin)
                {
                    selectedPoints.Add(i);
                }
            }

            int numOfPoints = (int)(allPoints.Count * _initalPercentPoints / 100f);

            var startTime = DateTime.UtcNow;
            int totalSecs = 0;

            while (selectedPoints.Count < numOfPoints)
            {
                double maxDistance = 0;
                int idForNextPoint = 0;
                int i = 0;
                foreach (var point in allPoints) // перебор всех точек и еще и не один раз
                {
                    // Пропускаем уже выбранные точки
                    if (selectedPoints.Contains(i))
                    {
                        i++;
                        continue;
                    }

                    // ко всем не выбранным точкам ищем ближаюшую выбранную

                    // Находим самую близкую точку среди выбранных к проверяемой (point)
                    double minDistance = double.MaxValue;
                    foreach (var selectedPoint in selectedPoints)
                    {
                        var timeNow = DateTime.UtcNow;
                        if ((timeNow - startTime).TotalSeconds >= 1.0)
                        {
                            totalSecs++;
                            Console.WriteLine($"Creating initial optimal set of points is in progress {totalSecs} seconds.");
                            startTime = timeNow;
                        }

                        double distance = Math.Sqrt(Math.Pow(point.Item1.X - allPoints[selectedPoint].Item1.X, 2) + Math.Pow(point.Item1.Y - allPoints[selectedPoint].Item1.Y, 2));
                        if (distance < minDistance)
                            minDistance = distance;
                    }

                    // Если расстояние до ближайшей выбранной точки больше максимального,
                    // то выбираем эту точку
                    if (minDistance > maxDistance)
                    {
                        maxDistance = minDistance;
                        idForNextPoint = i;
                    }

                    i++;
                }

                // Добавляем найденную точку в выбранные
                selectedPoints.Add(idForNextPoint);
            }

            for (int i = 0; i < selectedPoints.Count; i++)
            {
                var @new = (allPoints[selectedPoints[i]].Item1, allPoints[selectedPoints[i]].Item2, new QuadMeasurement(-1, -1, -1, -1, -1, -1));
                finalArrayWithInitialPoints.Add(@new);
            }

            selectedPoints.Sort();

            for (int i = selectedPoints.Count - 1; i >= 0; i--)
            {
                allPoints.RemoveAt(selectedPoints[i]);
            }
        }

        /// <summary>
        /// Метод меняет данные листа dataWithQuadruples!
        /// Отправляет данные из dataWithQuadruples на измерение в оборудование, если dataWithQuadruples[i].Item3.Resistance не подсчитан
        /// Дополняет dataWithQuadruples подсчитанными данными.
        /// </summary>
        /// <param name="dataWithQuadruples">координаты точки, квадруполь, данные о измерении</param>
        private void StartMeasurement(List<(Vector2, Quadruple, QuadMeasurement)> dataWithQuadruples, int numOfIteration)
        {
            var quads = new HashSet<Quadruple>();
            foreach (var data in dataWithQuadruples)
            {
                if (data.Item3.Resistance == -1)
                {
                    quads.Add(data.Item2);
                }
            }

            var measurements = new Dictionary<Quadruple, QuadMeasurement>();

            var positions = _measurementConfigs[0].Positions;

            // это лишь ссылка на функцию, она будет исполняться внутри вызова s.Measure
            NewDataHandler handler =
                (quadruples, outputCurrent, outputVoltage, channel, pulseInputVoltage, pauseInputVoltage,
                    coefficientOfVariation, idx_pack, count_pack) =>
                {
                    for (int i = 0; i < quadruples.Length; i++)
                    {
                        var q = quadruples[i];

                        var posA = positions[(ushort)q.A];
                        var posB = positions[(ushort)q.B];
                        var posM = positions[(ushort)q.M];
                        var posN = positions[(ushort)q.N];

                        var gFactor = CalcGeometryFactor(q, _options.ElectrodeStep, posA, posB, posM, posN);
                        var resistance = pulseInputVoltage[i] / outputCurrent[i] * Math.Abs(gFactor);
                        string format_pos = "0.000";
                        if (resistance < 0)
                        {
                            Console.WriteLine($"TROUBLE WITH THE DEVICE - resistance is less than 0:({idx_pack}/{count_pack}) A:{q.A} B:{q.B} M:{q.M} N:{q.N} ({resistance.ToString(format_pos)})");
                            resistance = 0; // по хорошему бы вообще останавливать процесс мониторинга
                            // приравниваем к нулю, чтобы значение было видно на логарифмической шкале и далее мы могли ручками его отмести
                        }
                        else
                        {
                            Console.WriteLine(
                            $"({idx_pack}/{count_pack}) A:{q.A} B:{q.B} M:{q.M} N:{q.N} ({resistance.ToString(format_pos)})");
                        }
                        measurements[q] = new QuadMeasurement(outputVoltage[i], outputCurrent[i], pulseInputVoltage[i],
                            pauseInputVoltage[i], coefficientOfVariation[i], (float)resistance);
                        Console.Out.Flush();
                    }

                    return true;
                };

            _toolERT.Measure(quads.ToArray(), handler, ParseResistanceData(_options));

            SaveMeasuredDataToFile(dataWithQuadruples, positions, measurements, numOfIteration);
        }

        public static bool[] ParseResistanceData(Options options)
        {
            XmlSerializer serializer = new XmlSerializer(typeof(bool[]));
            bool[] resistanceDataEnabled;
            using (XmlReader reader = XmlReader.Create(options.ResistanceData))
            {
                resistanceDataEnabled = (bool[])serializer.Deserialize(reader);
            }
            return resistanceDataEnabled;
        }

        public static double CalcGeometryFactor(Quadruple quadruple, float electrodeStep,
    Position posA, Position posB, Position posM, Position posN)
        {
            // WARNING!: Работает в предположении, что шаг между электродами в сиквенсе единичный

            var ertQuadruple = new ERT3D_Quadruple(
                quadruple.A,
                quadruple.B,
                quadruple.M,
                quadruple.N,
                new Point2D(posA.X * electrodeStep, posA.Y * electrodeStep),
                new Point2D(posB.X * electrodeStep, posB.Y * electrodeStep),
                new Point2D(posM.X * electrodeStep, posM.Y * electrodeStep),
                new Point2D(posN.X * electrodeStep, posN.Y * electrodeStep));

            return ertQuadruple.GeometryFactor_K;
        }

        private void SaveMeasuredDataToFile(List<(Vector2, Quadruple, QuadMeasurement)> dataWithQuadruples,
            Dictionary<ushort, Position> positions,
            Dictionary<Quadruple, QuadMeasurement> measurements,
            int numOfIteration)
        {
            var filePath = _options.OutputDir + "/" + _options.Prefix;
            var brokenId = filePath.IndexOf(".broken");
            if (brokenId != -1)
            {
                filePath = filePath[..brokenId] + "_" + numOfIteration.ToString() + ".broken";
            }
            else
            {
                filePath += "_" + numOfIteration.ToString();
            }
            filePath += "." + _measurementConfigs[0].MeasurementName + ".csv";
            FileStream fs = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            using StreamWriter sw = new StreamWriter(fs);
            sw.WriteLine(
                "A[idx];B[idx];M[idx];N[idx];Ax[m];Ay[m];Bx[m];By[m];Mx[m];My[m];Nx[m];Ny[m];Ox[m];Oy[m];ApparentDepth[m];" +
                "OutputCurrent[mA];OutputVoltage[V];PulseInputVoltage[mV];PauseInputVoltage[mV];ApparentResistivity[ohmm];CoefficientOfVariation[%]");
            for (int i = 0; i < dataWithQuadruples.Count; i++)
            {
                var posA = positions[(ushort)dataWithQuadruples[i].Item2.A];
                var posB = positions[(ushort)dataWithQuadruples[i].Item2.B];
                var posM = positions[(ushort)dataWithQuadruples[i].Item2.M];
                var posN = positions[(ushort)dataWithQuadruples[i].Item2.N];
                string format_pos = "0.000";
                double Ox = 0, Oy = 0, ApparentDepth = 0;
                QuadMeasurement measured;
                if (dataWithQuadruples[i].Item3.Resistance == -1f)
                {
                    measured = measurements[dataWithQuadruples[i].Item2];
                    var newItem = (dataWithQuadruples[i].Item1, dataWithQuadruples[i].Item2, measured);
                    dataWithQuadruples[i] = newItem;
                }
                else
                {
                    measured = dataWithQuadruples[i].Item3;
                }

                var strToPrint =
                        $"{dataWithQuadruples[i].Item2.A};{dataWithQuadruples[i].Item2.B};{dataWithQuadruples[i].Item2.M};{dataWithQuadruples[i].Item2.N};{posA.X.ToString(format_pos)};{posA.Y.ToString(format_pos)};{posB.X.ToString(format_pos)};{posB.Y.ToString(format_pos)};{posM.X.ToString(format_pos)};" +
                        $"{posM.Y.ToString(format_pos)};{posN.X.ToString(format_pos)};{posN.Y.ToString(format_pos)};{Ox.ToString(format_pos)};{Oy.ToString(format_pos)};{ApparentDepth.ToString(format_pos)};{measured.OutputCurrent};{measured.OutputVoltage};" +
                        $"{measured.PulseInputVoltage};{measured.PauseInputVoltage};" +
                        $"{measured.Resistance};{measured.CoefficientOfVariation}";
                sw.WriteLine(strToPrint);
            }
        }

        /// <summary>
        /// По значениям ABMN и _options.ElectrodeStep определяет координаты точек, которые будут в графе.
        /// </summary>
        /// <returns>Вернет (X координата, Y координата(глубина), A, B, M, N)</returns>
        private List<(Vector2, Quadruple)> GetDataWithPointsCoordinates()
        {
            var quads = new HashSet<Quadruple>();

            if (_measurementConfigs.Length != 1)
            {
                Console.WriteLine($"\nSequence has more than one section (example: schlumberger + dipole-dipole)\nWill be used only first one section.");
                Console.Out.Flush();
            }


            foreach (var configQuadruple in _measurementConfigs[0].Quadruples)
            {
                quads.Add(configQuadruple);
            }

            var yList = new List<ERT3D_Quadruple>();
            foreach (var quad in quads)
            {
                ERT3D_Quadruple quadruple;
                quadruple = new ERT3D_Quadruple(
                    quad.A,
                    quad.B,
                    quad.M,
                    quad.N,
                new Point2D((quad.A - 1) * _options.ElectrodeStep, 0),
                new Point2D((quad.B - 1) * _options.ElectrodeStep, 0),
                new Point2D((quad.M - 1) * _options.ElectrodeStep, 0),
                new Point2D((quad.N - 1) * _options.ElectrodeStep, 0),
                indexRemote_1: null,
                indexRemote_2: null
            );
                yList.Add(quadruple);
            }

            ERT3D_ComputeMeasurementPointPos.ComputeMeasurementPointPosMultipleConcurrentNoLog(yList, _options.ElectrodeStep);

            var array = new List<(Vector2, Quadruple)>();
            for (int j = 0; j < yList.Count; j++)
            {
                var quad = new Quadruple();
                quad.A = yList[j].IndexA;
                quad.B = yList[j].IndexB;
                quad.M = yList[j].IndexM;
                quad.N = yList[j].IndexN;
                array.Add((new Vector2((float)yList[j].MeasurementPointXYZ.X, (float)yList[j].MeasurementPointXYZ.Z), quad));
            }

            return array;
        }

        // метод для отладки и визуализации в проекте ViewERT.WPF_tests
        private void WritePointsToFile(List<(Vector2, Quadruple, QuadMeasurement)> points)
        {
            var fullPathToPoints = "C:\\Users\\nikit\\Documents\\GitHub\\StorageERT\\ViewERT.WPF_tests\\bin\\Debug\\net6.0-windows\\input\\PointsData.txt";
            using (StreamWriter writer = new StreamWriter(fullPathToPoints))
            {
                const float multiplier = 30.0f; // чтобы было лучше видно
                for (int i = 0; i < points.Count; i++)
                {
                    writer.WriteLine((points[i].Item1.X * multiplier).ToString() + ' ' + Math.Abs(points[i].Item1.Y * multiplier).ToString());
                }
            }
        }
    }
}
