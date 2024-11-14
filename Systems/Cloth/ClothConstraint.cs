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
        // Solve for the constraint equation C(x) = 0
        Equal,
        // Solve for the constraint equation C(x) < 0
        LessThan,
        // Solve for the constraint equation C(x) > 0
        GreaterThan,
    }
   
    [ProtoContract]
    public class ClothConstraint
    {
        [ProtoMember(1)]
        public int[] PointIndices;
        [ProtoMember(2)]
        public double Compliance;
        [ProtoMember(3)]
        // Generic parameter that might be interpreted differently
        // by different types of constraints
        // - Distance constraint: target distance between points
        // - Angle constraint: target angle between lines
        public double Param1;
        [ProtoMember(4)]
        public ConstraintType Type;
        [ProtoMember(5)]
        public ConstraintComparison Comparison;
        [ProtoMember(6)]
        public double Damping;

        ClothPoint[] points;
        public ClothPoint Point1 => points[0];
        public ClothPoint Point2 => points[1];
        public ClothPoint Point3 => points[2];

        // float rest_length;
        // float inverse_length;
        // Vec3f tensionDirection = new Vec3f();
        // private float StretchStiffness = 200f;
        // double springLength;
        public Vec3d renderCenterPos;
        // double extension;

        // public double LagrangeMultiplier;

        // Different interpretations of Param value,
        // just for the sake of readability
        public double TargetDistance { get { return Param1; } set { Param1 = value; } }
        public double TargetAngleRad { get { return Param1; } set { Param1 = value; } }

        public ClothConstraint()
        {

        }

        // TODO: do a factory instead?
        public ClothConstraint(ClothPoint[] points, ConstraintType type, double compliance, ConstraintComparison comparison = ConstraintComparison.Equal)
        {
            switch (type)
            {
            case ConstraintType.Distance:
            {
                if (points.Length != 2) throw new Exception("Incorrect constraint parameters");
                break;
            }
            case ConstraintType.Angle:
            {
                if (points.Length != 3) throw new Exception("Incorrect constraint parameters");
                break;
            }
            default:
                break;
            }

            this.Type = type;
            this.points = points;
            this.Compliance = 0.01;//compliance;
            this.Damping = 0.5;
            this.Comparison = comparison;

            PointIndices = new int[points.Length];
            renderCenterPos = Vec3d.Zero;

            for (int i = 0; i < points.Length; i++)
            {
                PointIndices[i] = points[i].PointIndex;
                renderCenterPos += points[i].Pos;
            }

            renderCenterPos /= points.Length;

            // renderCenterPos = p1.Pos + (p1.Pos - p2.Pos) / 2;
        }

        public void RestorePoints(Dictionary<int, ClothPoint> pointsByIndex)
        {
            points = new ClothPoint[PointIndices.Length];

            for (int i = 0; i < PointIndices.Length; i++)
            {
                points[i] = pointsByIndex[PointIndices[i]];
            }
            // renderCenterPos = p1.Pos + (p1.Pos - p2.Pos) / 2;
        }

        public void CalcDistanceConstraint(out double value, out Vec3d gradientPoint1, out Vec3d gradientPoint2)
        {
            const double epsilon = 0.000001;

            // Distance between two points:
            //
            // C = |p1 - p2| - target_dist
            //
            var diff = (points[0].Pos - points[1].Pos);
            var length = diff.Length();

            value = length - TargetDistance;

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            if (length > epsilon) {
                gradientPoint1 = diff / length;
            } else {
                gradientPoint1 = new Vec3d(1.0, 0.0, 0.0);
            }

            // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
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
            // C' = acos((p1 - p2) * (p2 - p3)) - phi

            var vec1 = (points[0].Pos - points[1].Pos).Normalize();
            var vec2 = (points[1].Pos - points[3].Pos).Normalize();

            var dotProduct = vec1.Dot(vec2);

            value = Math.Acos(dotProduct) - TargetAngleRad;

            // TODO: this is wrong
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

        public void Update(TimeStepData time)
        {
            switch (Type)
            {
            case ConstraintType.Distance:
            {
                double constraintValue;
                Vec3d gradientP1;
                Vec3d gradientP2;

                CalcDistanceConstraint(out constraintValue, out gradientP1, out gradientP2);

                double alpha = Compliance * time.InvSubstepTimeSqr;

                double gamma_u = alpha * Damping;
                double dampingP1 = gamma_u * (gradientP1.Dot(points[0].Pos - points[0].PrevPos));
                double dampingP2 = gamma_u * (gradientP2.Dot(points[1].Pos - points[1].PrevPos));

                double lambda_delta =
                    ( -constraintValue - dampingP1 - dampingP2) /
                    ( (1.0 + gamma_u * time.InvSubstepTime) *
                      (gradientP1.LengthSq() * points[0].InvMass +
                       gradientP2.LengthSq() * points[1].InvMass) +
                      alpha );

                // LagrangeMultiplier += lambda_delta;

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

                double alpha = Compliance * time.InvSubstepTimeSqr;

                double lambda_delta =
                    ( -constraintValue ) /
                    ( gradientP1.LengthSq() * points[0].InvMass +
                      gradientP2.LengthSq() * points[1].InvMass +
                      gradientP3.LengthSq() * points[2].InvMass +
                      alpha );

                // LagrangeMultiplier += lambda_delta;

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
    }
}
