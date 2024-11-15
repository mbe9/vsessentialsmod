using ProtoBuf;
using System;
using System.Collections.Generic;
using Vintagestory.API.MathTools;
using Vintagestory.API.Client;
using System.Runtime.CompilerServices;

namespace Vintagestory.GameContent
{
    [ProtoContract]
    public enum ConstraintComparison : byte
    {
        // Solve for the constraint equation C(x) = 0
        //
        // Use when you need to match target value exactly,
        // for example maintain distance between points at length L
        Equal,
        // Solve for the constraint equation C(x) < 0
        //
        // Use when you need to keep actual values of the constraint less than target value,
        // for example maintain angle between two lines as always less than some angle A
        LessThan,
        // Solve for the constraint equation C(x) > 0
        //
        // Use when you need to keep actual values of the constraint greater than target value,
        // for example maintain angle between two lines as always bigger than some angle A
        GreaterThan,
    }

    public struct LineSegemntCollisionConstraint
    {
        public float Friction;

        public ushort PointIndex1;
        public ushort PointIndex2;

        public void Update(Span<ClothPointData> points, TimeStepData time)
        {


        }
    }

    // Note: we don't serialize this struct since collision constraints
    // are constructed from scratch every frame
    public struct CollisionConstraint
    {
        public FastVec3d collisionBoxPlanes;

        public float Friction;
        public ushort PointIndex;

        public byte planeDirs;

        public void Update(Span<ClothPointData> points, TimeStepData time)
        {
            const double epsilon = 0.00001;

            ref var p = ref points[PointIndex];

            if (p.Movable == 0) return;

            FastVec3d gradient;

            (double constraintValue, char moveDir) = CalcCollisionConstraint(in p.Pos, out gradient);

            if (constraintValue > 0.0) return;

            double delta;
            double deltaBorder;

            switch (moveDir)
            {
            case 'X':
                delta = p.Pos.X - p.PrevPos.X;
                deltaBorder = (p.Pos.X - gradient.X * constraintValue) - p.PrevPos.X;
                break;
            case 'Y':
                delta = p.Pos.Y - p.PrevPos.Y;
                deltaBorder = (p.Pos.Y - gradient.Y * constraintValue) - p.PrevPos.Y;
                break;
            case 'Z':
                delta = p.Pos.Z - p.PrevPos.Z;
                deltaBorder = (p.Pos.Z - gradient.Z * constraintValue) - p.PrevPos.Z;
                break;
            default:
                throw new System.Diagnostics.UnreachableException();
            }

            double t = (Math.Abs(delta) > epsilon) ? (deltaBorder / delta) : 0.5;

            t = Math.Clamp(t, 0.0, 1.0);

            // Point of collision for the line going from previous to current position
            // This point is the final result if the friction is maximum (1.0)
            FastVec3d collisionPoint;
            collisionPoint.X = p.Pos.X * t + p.PrevPos.X * (1.0 - t);
            collisionPoint.Y = p.Pos.Y * t + p.PrevPos.Y * (1.0 - t);
            collisionPoint.Z = p.Pos.Z * t + p.PrevPos.Z * (1.0 - t);

            // Result of extruding current position along collision box normal
            // This point is the final result if there is no friction (0.0)
            FastVec3d slidePoint;
            slidePoint.X = p.Pos.X - gradient.X * constraintValue;
            slidePoint.Y = p.Pos.Y - gradient.Y * constraintValue;
            slidePoint.Z = p.Pos.Z - gradient.Z * constraintValue;

            p.Pos.X = slidePoint.X * (1.0 - Friction) + collisionPoint.X * Friction;
            p.Pos.Y = slidePoint.Y * (1.0 - Friction) + collisionPoint.Y * Friction;
            p.Pos.Z = slidePoint.Z * (1.0 - Friction) + collisionPoint.Z * Friction;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public (double, char) CalcCollisionConstraint(in FastVec3d pos, out FastVec3d gradient)
        {
            FastVec3d planeDirections;
            planeDirections.X = (double)(2 * (sbyte)((planeDirs) & 1) - 1);
            planeDirections.Y = (double)(2 * (sbyte)((planeDirs >> 1) & 1) - 1);
            planeDirections.Z = (double)(2 * (sbyte)((planeDirs >> 2) & 1) - 1);

            FastVec3d delta;
            delta.X = (pos.X - collisionBoxPlanes.X);
            delta.Y = (pos.Y - collisionBoxPlanes.Y);
            delta.Z = (pos.Z - collisionBoxPlanes.Z);

            FastVec3d dir_delta;
            dir_delta.X = delta.X * planeDirections.X;
            dir_delta.Y = delta.Y * planeDirections.Y;
            dir_delta.Z = delta.Z * planeDirections.Z;

            if ((dir_delta.X > dir_delta.Y) && (dir_delta.X > dir_delta.Z))
            {
                gradient.X = planeDirections.X;
                gradient.Y = 0;
                gradient.Z = 0;

                return (dir_delta.X, 'X');
            }
            else if ((dir_delta.Y > dir_delta.X) && (dir_delta.Y > dir_delta.Z))
            {
                gradient.X = 0;
                gradient.Y = planeDirections.Y;
                gradient.Z = 0;

                return (dir_delta.Y, 'Y');
            }
            else
            {
                gradient.X = 0;
                gradient.Y = 0;
                gradient.Z = planeDirections.Z;

                return (dir_delta.Z, 'Z');
            }
        }
    }


    [ProtoContract]
    public struct LengthConstraint
    {
        [ProtoMember(1)]
        public float Compliance;
        [ProtoMember(2)]
        public float Damping;
        [ProtoMember(3)]
        public float TargetLength;
        [ProtoMember(4)]
        public ConstraintComparison Comparison;

        public void Update(Span<ClothPointData> points, TimeStepData time)
        {
            if (points.Length == 0) return;

            double alpha = (double)Compliance * time.InvSubstepTimeSqr;
            double gamma_u = alpha * (double)Damping;

            var gradients = new FastVec3d[2 * (points.Length - 1)];
            double sumLength = 0.0;
            double sumInvMass = 0.0;
            double dampingSum = 0.0;
            byte sumMovable = 0;

            for (int i = 0; i < points.Length - 1; i++)
            {
                ref var p1 = ref points[i];
                ref var p2 = ref points[i + 1];

                int gradient1Idx = i * 2;
                int gradient2Idx = i * 2 + 1;

                sumLength += CalcDistanceConstraint(in p1.Pos,
                                                    in p2.Pos,
                                                    out gradients[gradient1Idx],
                                                    out gradients[gradient2Idx]);

                sumInvMass += p1.InvMass * (double)p1.Movable;

                sumMovable |= p1.Movable;

                dampingSum +=
                    (p1.Pos.X - p1.PrevPos.X) * gradients[gradient1Idx].X +
                    (p1.Pos.Y - p1.PrevPos.Y) * gradients[gradient1Idx].Y +
                    (p1.Pos.Z - p1.PrevPos.Z) * gradients[gradient1Idx].Z +

                    (p2.Pos.X - p2.PrevPos.X) * gradients[gradient2Idx].X +
                    (p2.Pos.Y - p2.PrevPos.Y) * gradients[gradient2Idx].Y +
                    (p2.Pos.Z - p2.PrevPos.Z) * gradients[gradient2Idx].Z;
            }

            dampingSum *= gamma_u;

            // Handle last point
            sumInvMass += points[points.Length - 1].InvMass;
            sumMovable |= points[points.Length - 1].Movable;


            double constraintValue = sumLength - TargetLength;

            switch (Comparison)
            {
            case ConstraintComparison.Equal:
                break;
            case ConstraintComparison.LessThan:
                if (constraintValue < 0.0) return;
                break;
            case ConstraintComparison.GreaterThan:
                if (constraintValue > 0.0) return;
                break;
            }

            if (sumMovable == 0) return;

            double lambda_delta =
                ( - constraintValue - dampingSum ) /
                ( alpha + (1.0 + gamma_u * time.InvSubstepTime) * sumInvMass );

            for (int i = points.Length - 2; i >= 0; i--)
            {
                ref var p1 = ref points[i];
                ref var p2 = ref points[i + 1];

                int gradient1Idx = i * 2;
                int gradient2Idx = i * 2 + 1;

                ref var gradientP1 = ref gradients[gradient1Idx];
                ref var gradientP2 = ref gradients[gradient2Idx];

                double isMovableP1 = (double)p1.Movable;
                double isMovableP2 = (double)p2.Movable;

                p1.Pos.X += gradientP1.X * (p1.InvMass * lambda_delta * isMovableP1);
                p1.Pos.Y += gradientP1.Y * (p1.InvMass * lambda_delta * isMovableP1);
                p1.Pos.Z += gradientP1.Z * (p1.InvMass * lambda_delta * isMovableP1);

                p2.Pos.X += gradientP2.X * (p2.InvMass * lambda_delta * isMovableP1);
                p2.Pos.Y += gradientP2.Y * (p2.InvMass * lambda_delta * isMovableP1);
                p2.Pos.Z += gradientP2.Z * (p2.InvMass * lambda_delta * isMovableP1);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double CalcDistanceConstraint(in FastVec3d pos1,
                                             in FastVec3d pos2,
                                             out FastVec3d gradient1,
                                             out FastVec3d gradient2)
        {
            const double epsilon = 0.0001;

            // Distance between two points:
            //
            // C = |p1 - p2| - target_dist
            //
            FastVec3d diff;
            diff.X = pos1.X - pos2.X;
            diff.Y = pos1.Y - pos2.Y;
            diff.Z = pos1.Z - pos2.Z;

            double length = diff.Length();
            double inv_length = 1.0 / length;

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            if (length > epsilon)
            {
                gradient1.X = diff.X * inv_length;
                gradient1.Y = diff.Y * inv_length;
                gradient1.Z = diff.Z * inv_length;
            }
            else
            {
                gradient1.X = 1.0;
                gradient1.Y = 0.0;
                gradient1.Z = 0.0;
            }

            // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
            gradient2.X = -gradient1.X;
            gradient2.Y = -gradient1.Y;
            gradient2.Z = -gradient1.Z;

            return length;
        }
    }

    [ProtoContract]
    public struct DistanceConstraint
    {
        [ProtoMember(1)]
        public float Compliance;
        [ProtoMember(2)]
        public float Damping;
        [ProtoMember(3)]
        public float TargetDistance;
        [ProtoMember(4)]
        public ushort PointIndex1;
        [ProtoMember(5)]
        public ushort PointIndex2;
        [ProtoMember(6)]
        public ConstraintComparison Comparison;

        public void Update(Span<ClothPointData> points, TimeStepData time)
        {
            ref var p1 = ref points[PointIndex1];
            ref var p2 = ref points[PointIndex2];

            if ((p1.Movable == 0) && (p2.Movable == 0)) return;

            FastVec3d gradientP1;
            FastVec3d gradientP2;

            double constraintValue = CalcDistanceConstraint(in p1.Pos, in p2.Pos,
                                                            out gradientP1, out gradientP2);

            switch (Comparison)
            {
            case ConstraintComparison.Equal:
                break;
            case ConstraintComparison.LessThan:
                if (constraintValue < 0.0) return;
                break;
            case ConstraintComparison.GreaterThan:
                if (constraintValue > 0.0) return;
                break;
            }

            double alpha = (double)Compliance * time.InvSubstepTimeSqr;
            double gamma_u = alpha * (double)Damping;

            double damping = gamma_u * (
                (p1.Pos.X - p1.PrevPos.X) * gradientP1.X +
                (p1.Pos.Y - p1.PrevPos.Y) * gradientP1.Y +
                (p1.Pos.Z - p1.PrevPos.Z) * gradientP1.Z +

                (p2.Pos.X - p2.PrevPos.X) * gradientP2.X +
                (p2.Pos.Y - p2.PrevPos.Y) * gradientP2.Y +
                (p2.Pos.Z - p2.PrevPos.Z) * gradientP2.Z
            );

            double effectiveInvMassP1 = (double)p1.Movable * p1.InvMass;
            double effectiveInvMassP2 = (double)p2.Movable * p2.InvMass;

            double lambda_delta =
                ( -constraintValue - damping ) /
                ( alpha + (1.0 + gamma_u * time.InvSubstepTime) * (effectiveInvMassP1 + effectiveInvMassP2) );

            // LagrangeMultiplier += lambda_delta;

            // Since we are using Verlet integration, simply updating position here
            // also implicitly updates the velocity of the points

            p1.Pos.X += gradientP1.X * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Y += gradientP1.Y * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Z += gradientP1.Z * (lambda_delta * effectiveInvMassP1);

            p2.Pos.X += gradientP2.X * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Y += gradientP2.Y * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Z += gradientP2.Z * (lambda_delta * effectiveInvMassP2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double CalcDistanceConstraint(in FastVec3d pos1,
                                             in FastVec3d pos2,
                                             out FastVec3d gradient1,
                                             out FastVec3d gradient2)
        {
            const double epsilon = 0.0001;

            // Distance between two points:
            //
            // C = |p1 - p2| - target_dist
            //
            FastVec3d diff;
            diff.X = pos1.X - pos2.X;
            diff.Y = pos1.Y - pos2.Y;
            diff.Z = pos1.Z - pos2.Z;

            double length = diff.Length();
            double inv_length = 1.0 / length;

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            if (length > epsilon)
            {
                gradient1.X = diff.X * inv_length;
                gradient1.Y = diff.Y * inv_length;
                gradient1.Z = diff.Z * inv_length;
            }
            else
            {
                gradient1.X = 1.0;
                gradient1.Y = 0.0;
                gradient1.Z = 0.0;
            }

            // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
            gradient2.X = -gradient1.X;
            gradient2.Y = -gradient1.Y;
            gradient2.Z = -gradient1.Z;

            return length - (double)TargetDistance;
        }
    }

    [ProtoContract]
    public struct AngleConstraint
    {
        [ProtoMember(1)]
        public float Compliance;
        [ProtoMember(2)]
        public float Damping;
        // Important to note that this is not the target angle itself,
        // but rather Cosine of the desired angle.
        // Using Cosine allows us to simplify constraint calculations significantly
        [ProtoMember(3)]
        public float TargetAngleCos;
        [ProtoMember(4)]
        public ushort PointIndex1;
        [ProtoMember(5)]
        public ushort PointIndex2;
        [ProtoMember(6)]
        public ushort PointIndex3;
        [ProtoMember(7)]
        public ConstraintComparison Comparison;

        public void Update(Span<ClothPointData> points, TimeStepData time)
        {
            ref var p1 = ref points[PointIndex1];
            ref var p2 = ref points[PointIndex2];
            ref var p3 = ref points[PointIndex3];

            if ((p1.Movable == 0) && (p2.Movable == 0) && (p3.Movable == 0)) return;

            FastVec3d gradientP1;
            FastVec3d gradientP2;
            FastVec3d gradientP3;

            (double constraintValue, bool valid) = CalcAngleConstraint(in p1.Pos, in p2.Pos, in p3.Pos,
                                                                       out gradientP1, out gradientP2, out gradientP3);

            // Do not attempt any further calculations if we could not
            // calculate the constraint values
            if (!valid) return;

            switch (Comparison)
            {
            case ConstraintComparison.Equal:
                break;
            case ConstraintComparison.LessThan:
                if (constraintValue < 0.0) return;
                break;
            case ConstraintComparison.GreaterThan:
                if (constraintValue > 0.0) return;
                break;
            }

            double alpha = (double)Compliance * time.InvSubstepTimeSqr;
            double gamma_u = alpha * (double)Damping;

            double damping =
                gamma_u * (
                    (p1.Pos.X - p1.PrevPos.X) * gradientP1.X +
                    (p1.Pos.Y - p1.PrevPos.Y) * gradientP1.Y +
                    (p1.Pos.Z - p1.PrevPos.Z) * gradientP1.Z +

                    (p2.Pos.X - p2.PrevPos.X) * gradientP2.X +
                    (p2.Pos.Y - p2.PrevPos.Y) * gradientP2.Y +
                    (p2.Pos.Z - p2.PrevPos.Z) * gradientP2.Z +

                    (p3.Pos.X - p3.PrevPos.X) * gradientP3.X +
                    (p3.Pos.Y - p3.PrevPos.Y) * gradientP3.Y +
                    (p3.Pos.Z - p3.PrevPos.Z) * gradientP3.Z
                );

            double effectiveInvMassP1 = (double)p1.Movable * p1.InvMass;
            double effectiveInvMassP2 = (double)p2.Movable * p2.InvMass;
            double effectiveInvMassP3 = (double)p3.Movable * p3.InvMass;

            double lambda_delta =
                ( -constraintValue - damping ) /
                ( alpha + (1.0 + gamma_u * time.InvSubstepTime) *
                  (gradientP1.LengthSq() * effectiveInvMassP1 +
                   gradientP2.LengthSq() * effectiveInvMassP2 +
                   gradientP3.LengthSq() * effectiveInvMassP3) );

            // LagrangeMultiplier += lambda_delta;

            // Since we are using Verlet integration, simply updating position here
            // also implicitly updates the velocity of the points

            p1.Pos.X += gradientP1.X * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Y += gradientP1.Y * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Z += gradientP1.Z * (lambda_delta * effectiveInvMassP1);

            p2.Pos.X += gradientP2.X * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Y += gradientP2.Y * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Z += gradientP2.Z * (lambda_delta * effectiveInvMassP2);

            p3.Pos.X += gradientP3.X * (lambda_delta * effectiveInvMassP3);
            p3.Pos.Y += gradientP3.Y * (lambda_delta * effectiveInvMassP3);
            p3.Pos.Z += gradientP3.Z * (lambda_delta * effectiveInvMassP3);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public (double, bool) CalcAngleConstraint(in FastVec3d pos1, in FastVec3d pos2, in FastVec3d pos3,
                                                  out FastVec3d gradient1, out FastVec3d gradient2, out FastVec3d gradient3)
        {
            const double epsilon = 0.0001;

            FastVec3d vec1;
            vec1.X = pos1.X - pos2.X;
            vec1.Y = pos1.Y - pos2.Y;
            vec1.Z = pos1.Z - pos2.Z;

            FastVec3d vec2;
            vec2.X = pos2.X - pos3.X;
            vec2.Y = pos2.Y - pos3.Y;
            vec2.Z = pos2.Z - pos3.Z;

            FastVec3d diff;
            diff.X = vec1.X - vec2.X;
            diff.Y = vec1.Y - vec2.Y;
            diff.Z = vec1.Z - vec2.Z;

            double vec1_length = vec1.Length();
            double vec2_length = vec2.Length();

            double ratioVec12 = vec1_length / vec2_length;
            double ratioVec21 = vec2_length / vec1_length;

            gradient1.X = vec2.X - vec1.X * (TargetAngleCos * ratioVec21);
            gradient1.Y = vec2.Y - vec1.Y * (TargetAngleCos * ratioVec21);
            gradient1.Z = vec2.Z - vec1.Z * (TargetAngleCos * ratioVec21);

            gradient2.X = (vec1.X - vec2.X) + TargetAngleCos * (vec1.X * ratioVec21 - vec2.X * ratioVec12);
            gradient2.Y = (vec1.Y - vec2.Y) + TargetAngleCos * (vec1.Y * ratioVec21 - vec2.Y * ratioVec12);
            gradient2.Z = (vec1.Z - vec2.Z) + TargetAngleCos * (vec1.Z * ratioVec21 - vec2.Z * ratioVec12);

            gradient3.X = -vec1.X + vec2.X * (TargetAngleCos * ratioVec12);
            gradient3.Y = -vec1.Y + vec2.Y * (TargetAngleCos * ratioVec12);
            gradient3.Z = -vec1.Z + vec2.Z * (TargetAngleCos * ratioVec12);

            double vecLengthMult = vec1_length * vec2_length;

            double constraintValue = (vec1.X * vec2.X) + (vec1.Y * vec2.Y) + (vec1.Z * vec2.Z) - ((double)TargetAngleCos * vecLengthMult);

            // Return (X, true) if calculations do not contain any divisions by zero
            // and return value is actually valid
            return (constraintValue, vecLengthMult > epsilon);
        }
    }
   
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public double CalcDistanceConstraint(in FastVec3d pos1,
        //                                      in FastVec3d pos2,
        //                                      out FastVec3d gradient1,
        //                                      out FastVec3d gradient2)
        // {
        //     const double epsilon = 0.000001;

        //     // Distance between two points:
        //     //
        //     // C = |p1 - p2| - target_dist
        //     //
        //     FastVec3d diff;
        //     diff.X = pos1.X - pos2.X;
        //     diff.Y = pos1.Y - pos2.Y;
        //     diff.Z = pos1.Z - pos2.Z;

        //     double length = diff.Length();
        //     double inv_length = 1.0 / length;

        //     // Derivative of distance between two points,
        //     // with respect to each of the points:
        //     //
        //     // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

        //     // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
        //     if (length > epsilon)
        //     {
        //         gradient1.X = diff.X * inv_length;
        //         gradient1.Y = diff.Y * inv_length;
        //         gradient1.Z = diff.Z * inv_length;
        //     }
        //     else
        //     {
        //         gradient1 = new FastVec3d(1.0, 0.0, 0.0);
        //     }

        //     // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
        //     gradient2.X = -gradient1.X;
        //     gradient2.Y = -gradient1.Y;
        //     gradient2.Z = -gradient1.Z;

        //     return length - TargetDistance;
        // }

        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public void CalcAngleConstraint(out double value,
        //                                 out FastVec3d gradient1,
        //                                 out FastVec3d gradient2,
        //                                 out FastVec3d gradient3)
        // {
        //     // Angle between two lines:
        //     //
        //     // (Here and further, * denotes dot product between 2 vectors)
        //     //
        //     // C' = acos((p1 - p2) * (p2 - p3)) - phi

        //     var vec1 = (points[0].Pos - points[1].Pos).Normalize();
        //     var vec2 = (points[1].Pos - points[3].Pos).Normalize();

        //     var dotProduct = vec1.Dot(vec2);

        //     value = Math.Acos(dotProduct) - TargetAngleRad;

        //     // TODO: this is wrong
        //     // Derivative of the angle between two lines,
        //     // with respect to each of the points
        //     //
        //     // d = acos((p1 - p2) * (p2 - p3))' =
        //     //   = - ((p1 - p2) * (p2 - p3))' / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =
        //     //   = - ((p1 - p2)' * (p2 - p3) + (p1 - p2) * (p2 - p3)') / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =

        //     var divisor = -1.0 / Math.Sqrt(1.0 - dotProduct * dotProduct);

        //     // d_p1 = - ((p1 - p2)' * (p2 - p3) + (p1 - p2) * (p2 - p3)') / sqrt(1 - ((p1 - p2) * (p2 - p3))^2) =
        //     //      = - ((1 - 0) * (p2 - p3) + (p1 - p2) * (0 - 0)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
        //     gradient1 = vec2 * divisor;

        //     // d_p2 = - ((0 - 1) * (p2 - p3) + (p1 - p2) * (1 - 0)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
        //     gradient2 = (vec1 - vec2) * divisor;

        //     // d_p3 = - ((0 - 0) * (p2 - p3) + (p1 - p2) * (0 - 1)) / sqrt(1 - ((p1 - p2) * (p2 - p3))^2)
        //     gradient3 = (Vec3d.Zero - vec1) * divisor; // TODO: negation operator would be nice to have
        // }
}
