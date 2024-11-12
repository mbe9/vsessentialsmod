using ProtoBuf;
using System;
using System.Collections.Generic;
using Vintagestory.API.MathTools;
using Vintagestory.API.Client;

namespace Vintagestory.GameContent
{
    [ProtoContract]
    public enum ConstraintType
    {
        Distance,
        Angle,
    }

    [ProtoContract]
    public enum ConstraintComparison
    {
        Equal,
        LessThan,
        GreaterThan,
    }
   
    [ProtoContract]
    public class ClothConstraint
    {
        static Random Rand = new Random();


        [ProtoMember(1)]
        public int[] PointIndices;
        [ProtoMember(2)]
        public int PointIndex2;
        [ProtoMember(3)]
        // Only used for angle constraint
        public int PointIndex3;
        [ProtoMember(4)]
        public float Compliance;
        [ProtoMember(5)]
        // Generic parameter that might be interpreted differently
        // by different types of constraints
        // - Distance constraint: target distance between points
        // - Angle constraint: target angle between lines
        public float Param1;
        [ProtoMember(6)]
        public ConstraintType Type;
        [ProtoMember(7)]
        public ConstraintComparison Comparison;


        [ProtoMember(3)]
        float squared_rest_length;


        ClothPoint[] points;
        public ClothPoint Point1 => points[0];
        public ClothPoint Point2 => points[1];
        public ClothPoint Point3 => points[2];

        static ParticlePhysics pp;
        static double TickTime = pp.PhysicsTickTime;
        static double InvTickTime = 1.0 / pp.PhysicsTickTime;
        static double InvTickTimeSqr = 1.0 / (pp.PhysicsTickTime * pp.PhysicsTickTime);

        float rest_length;
        float inverse_length;
        Vec3f tensionDirection = new Vec3f();
        private float StretchStiffness = 200f;
        double springLength;
        public Vec3d renderCenterPos;
        double extension;
        double lambda;


        public double LagrangeMultiplier => lambda;


        public double SpringLength => springLength;
        public double Extension => extension;


        public ClothConstraint()
        {

        }

        public ClothConstraint(ClothPoint p1, ClothPoint p2)
        {
            this.p1 = p1;
            this.p2 = p2;

            PointIndex1 = p1.PointIndex;
            PointIndex2 = p2.PointIndex;
            squared_rest_length = p1.Pos.SquareDistanceTo(p2.Pos);
            rest_length = GameMath.Sqrt(squared_rest_length);
            inverse_length = 1f / rest_length;

            renderCenterPos = p1.Pos + (p1.Pos - p2.Pos) / 2;
        }

        public void RestorePoints(Dictionary<int, ClothPoint> pointsByIndex)
        {
            p1 = pointsByIndex[PointIndex1];
            p2 = pointsByIndex[PointIndex2];

            rest_length = GameMath.Sqrt(squared_rest_length);
            inverse_length = 1f / rest_length;
            renderCenterPos = p1.Pos + (p1.Pos - p2.Pos) / 2;
        }

        public void CalcDistanceConstraint(out double value, out Vec3d gradientPoint1, out Vec3d gradientPoint2)
        {
            // Distance between two points:
            //
            // dist = |p1 - p2|
            //
            var diff = (points[0].Pos - points[1].Pos);

            value = diff.Length();

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // d = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // d_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            gradientPoint1 = diff / value;

            // d_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
            gradientPoint2 = Vec3d.Zero - gradientPoint1; // TODO: negation operator would be nice to have
        }

        public void CalcAngleConstraint(out double value,
                                        out Vec3d gradientPoint1,
                                        out Vec3d gradientPoint2,
                                        out Vec3d gradientPoint3)
        {
            // Angle between two lines:
            //
            // (Here and further, * denotes dot product between 2 vectors)
            //
            // phi = acos((p1 - p2) * (p2 - p3))

            var vec1 = points[0].Pos - points[1].Pos;
            var vec2 = points[1].Pos - points[3].Pos;

            var dotProduct = vec1.Dot(vec2);

            value = Math.Acos(dotProduct);

            // Derivative of the angle between two lines,
            // with respect to each of the points
            //
            // d = acos((p1 - p2) * (p2 - p3))' =
            //   = - ((p1 - p2) * (p2 - p3))' / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =
            //   = - ((p1 - p2)' * (p2 - p3) + (p1 - p2) * (p2 - p3)') / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =

            var divisor = -1.0 / Math.Sqrt(1.0 - dotProduct * dotProduct);

            // d_p1 = - ((p1 - p2)' * (p2 - p3) + (p1 - p2) * (p2 - p3)') / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =
            //      = - ((1 - 0) * (p2 - p3) + (p1 - p2) * (0 - 0)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
            gradientPoint1 = vec2 * divisor;

            // d_p2 = - ((0 - 1) * (p2 - p3) + (p1 - p2) * (1 - 0)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
            gradientPoint2 = (vec1 - vec2) * divisor;

            // d_p3 = - ((0 - 0) * (p2 - p3) + (p1 - p2) * (0 - 1)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
            gradientPoint3 = (Vec3d.Zero - vec1) * divisor; // TODO: negation operator would be nice to have
        }

        public void Update(float dt)
        {
            switch (Type)
            {
            case ConstraintType.Distance:
            {
                double constraintValue;
                Vec3d gradientP1;
                Vec3d gradientP2;

                CalcDistanceConstraint(out constraintValue, out gradientP1, out gradientP2);

                double alpha = Compliance * InvTickTimeSqr;

                double lambda_delta =
                    ( -constraintValue - alpha * lambda ) /
                    ( gradientP1.LengthSq() * points[0].InvMass + gradientP2.LengthSq() * points[1].InvMass + alpha );

                lambda += lambda_delta;

                // Since we are using Verlet integration, simply updating position here
                // also implicitly updates the velocity of the points

                points[0].Pos += points[0].InvMass * gradientP1 * lambda_delta;
                points[1].Pos += points[1].InvMass * gradientP2 * lambda_delta;
                break;
            }

            case ConstraintType.Angle:
            {
                double constraintValue;
                Vec3d gradientP1;
                Vec3d gradientP2;
                Vec3d gradientP3;

                CalcAngleConstraint(out constraintValue, out gradientP1, out gradientP2, out gradientP3);

                double alpha = Compliance * InvTickTimeSqr;

                double lambda_delta =
                    ( -constraintValue - alpha * lambda ) /
                    ( gradientP1.LengthSq() * points[0].InvMass +
                      gradientP2.LengthSq() * points[1].InvMass +
                      gradientP3.LengthSq() * points[2].InvMass +
                      alpha );

                lambda += lambda_delta;

                // Since we are using Verlet integration, simply updating position here
                // also implicitly updates the velocity of the points

                points[0].Pos += points[0].InvMass * gradientP1 * lambda_delta;
                points[1].Pos += points[1].InvMass * gradientP2 * lambda_delta;
                points[2].Pos += points[2].InvMass * gradientP3 * lambda_delta;
                break;
            }

            default:
                break;
            }
        }

        public void satisfy(float pdt)
        {
            tensionDirection.Set((float)(p1.Pos.X - p2.Pos.X), (float)(p1.Pos.Y - p2.Pos.Y), (float)(p1.Pos.Z - p2.Pos.Z));

            springLength = tensionDirection.Length();

            if (springLength == 0)
            {
                tensionDirection.Set((float)Rand.NextDouble() / 100f - 1/50f, (float)Rand.NextDouble() / 100f - 1 / 50f, (float)Rand.NextDouble() / 100f - 1 / 50f);
                springLength = tensionDirection.Length();
            }

            extension = springLength - rest_length;
            double tension = StretchStiffness * (extension * inverse_length);

            tensionDirection *= (float)(tension / springLength);

            p2.Tension.Add(tensionDirection);
            p1.Tension.Sub(tensionDirection);

            p2.TensionDirection.Set(tensionDirection);
            p1.TensionDirection.Set(-tensionDirection.X, -tensionDirection.Y, -tensionDirection.Z);

            p1.extension = extension;
            p2.extension = extension;
        }
    }
}
