using ProtoBuf;
using System;
using System.Collections.Generic;
using Vintagestory.API.MathTools;
using Vintagestory.API.Client;
using System.Runtime.CompilerServices;

namespace Vintagestory.GameContent.SoftBody
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

    // Note: we don't serialize this struct since collision constraints
    // are constructed from scratch every frame
    public struct LineSegemntCollisionConstraint
    {
        public FastVec3f minBoxBounds;
        public FastVec3f maxBoxBounds;

        public float Friction;

        public ushort PointIndex1;
        public ushort PointIndex2;

        public void Update(ClothPointData[] points, TimeStepData time, FastVec3f[] forces)
        {
            ref var p1 = ref points[PointIndex1];
            ref var p2 = ref points[PointIndex2];

            if ((p1.Movable == 0) && (p2.Movable == 0)) return;

            DualNum6 dualX1 = new DualNum6(p1.Pos.X, new[] {1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            DualNum6 dualY1 = new DualNum6(p1.Pos.Y, new[] {0.0, 1.0, 0.0, 0.0, 0.0, 0.0});
            DualNum6 dualZ1 = new DualNum6(p1.Pos.Z, new[] {0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
            DualNum6 dualX2 = new DualNum6(p2.Pos.X, new[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0});
            DualNum6 dualY2 = new DualNum6(p2.Pos.Y, new[] {0.0, 0.0, 0.0, 0.0, 1.0, 0.0});
            DualNum6 dualZ2 = new DualNum6(p2.Pos.Z, new[] {0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

            DualNum6 constraint;

            bool isColliding = CalcCollisionConstraint(in dualX1, in dualY1, in dualZ1,
                                                       in dualX2, in dualY2, in dualZ2,
                                                       out constraint);

            if (!isColliding) return;

            FastVec3d gradientP1;
            gradientP1.X = constraint[0];
            gradientP1.Y = constraint[1];
            gradientP1.Z = constraint[2];

            FastVec3d gradientP2;
            gradientP2.X = constraint[3];
            gradientP2.Y = constraint[4];
            gradientP2.Z = constraint[5];

            double effectiveInvMassP1 = p1.InvMass * (double)p1.Movable;
            double effectiveInvMassP2 = p2.InvMass * (double)p2.Movable;

            double lambda_delta =
                ( -constraint.value ) /
                ( (gradientP1.LengthSq() * effectiveInvMassP1 +
                   gradientP2.LengthSq() * effectiveInvMassP2) );

            FastVec3d deltaP1;
            deltaP1.X = gradientP1.X * (lambda_delta * effectiveInvMassP1);
            deltaP1.Y = gradientP1.Y * (lambda_delta * effectiveInvMassP1);
            deltaP1.Z = gradientP1.Z * (lambda_delta * effectiveInvMassP1);

            FastVec3d deltaP2;
            deltaP2.X = gradientP2.X * (lambda_delta * effectiveInvMassP2);
            deltaP2.Y = gradientP2.Y * (lambda_delta * effectiveInvMassP2);
            deltaP2.Z = gradientP2.Z * (lambda_delta * effectiveInvMassP2);

            // Friction calculations
            {

                FastVec3d frictionP1;
                frictionP1.X = Math.Abs(deltaP1.X) * Friction * 1000;
                frictionP1.Y = Math.Abs(deltaP1.Y) * Friction * 1000;
                frictionP1.Z = Math.Abs(deltaP1.Z) * Friction * 1000;

                p1.FrictionXY = Math.Max(p1.FrictionXY, (ushort)Math.Clamp(frictionP1.Z, ushort.MinValue, ushort.MaxValue));
                p1.FrictionXZ = Math.Max(p1.FrictionXZ, (ushort)Math.Clamp(frictionP1.Y, ushort.MinValue, ushort.MaxValue));
                p1.FrictionYZ = Math.Max(p1.FrictionYZ, (ushort)Math.Clamp(frictionP1.X, ushort.MinValue, ushort.MaxValue));

                FastVec3d frictionP2;
                frictionP2.X = Math.Abs(deltaP2.X) * Friction * 1000;
                frictionP2.Y = Math.Abs(deltaP2.Y) * Friction * 1000;
                frictionP2.Z = Math.Abs(deltaP2.Z) * Friction * 1000;

                p2.FrictionXY = Math.Max(p2.FrictionXY, (ushort)Math.Clamp(frictionP2.Z, ushort.MinValue, ushort.MaxValue));
                p2.FrictionXZ = Math.Max(p2.FrictionXZ, (ushort)Math.Clamp(frictionP2.Y, ushort.MinValue, ushort.MaxValue));
                p2.FrictionYZ = Math.Max(p2.FrictionYZ, (ushort)Math.Clamp(frictionP2.X, ushort.MinValue, ushort.MaxValue));
            }

            p1.Pos.X += deltaP1.X;
            p1.Pos.Y += deltaP1.Y;
            p1.Pos.Z += deltaP1.Z;

            p2.Pos.X += deltaP2.X;
            p2.Pos.Y += deltaP2.Y;
            p2.Pos.Z += deltaP2.Z;

            if (forces != null)
            {
                double invGradientP1 = 1 / gradientP1.Length();
                double invGradientP2 = 1 / gradientP2.Length();

                forces[PointIndex1].X += (float)(gradientP1.X * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Y += (float)(gradientP1.Y * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Z += (float)(gradientP1.Z * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));

                forces[PointIndex2].X += (float)(gradientP2.X * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Y += (float)(gradientP2.Y * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Z += (float)(gradientP2.Z * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
            }
        }

        // Liang–Barsky line clipping algorithm
        // implemented using Dual numbers for auto differentiation
        bool CalcCollisionConstraint(in DualNum6 x1, in DualNum6 y1, in DualNum6 z1,
                                     in DualNum6 x2, in DualNum6 y2, in DualNum6 z2,
                                     out DualNum6 result)
        {
            DualNum6 p1 = (x1 - x2);
            DualNum6 p2 = -p1;
            DualNum6 p3 = (y1 - y2);
            DualNum6 p4 = -p3;
            DualNum6 p5 = (z1 - z2);
            DualNum6 p6 = -p5;

            DualNum6 q1 = x1 - minBoxBounds.X;
            DualNum6 q2 = maxBoxBounds.X - x1;
            DualNum6 q3 = y1 - minBoxBounds.Y;
            DualNum6 q4 = maxBoxBounds.Y - y1;
            DualNum6 q5 = z1 - minBoxBounds.Z;
            DualNum6 q6 = maxBoxBounds.Z - z1;

            DualNum6 maxNegative = new DualNum6(0);
            DualNum6 minPositive = new DualNum6(1.0);

            const double epsilon = 0.0001;

            bool p1_zero = p1 < epsilon && p1 > -epsilon;
            bool p3_zero = p3 < epsilon && p3 > -epsilon;
            bool p5_zero = p5 < epsilon && p5 > -epsilon;

            bool parallelLineCheck =
                (p1_zero && q1 < 0) ||
                (p1_zero && q2 < 0) ||
                (p3_zero && q3 < 0) ||
                (p3_zero && q4 < 0) ||
                (p5_zero && q5 < 0) ||
                (p5_zero && q6 < 0);

            if (parallelLineCheck)
            {
                result = maxNegative; // assign garbage to appease compiler
                return false;
            }

            if (!p1_zero)
            {
                DualNum6 r1 = q1 / p1;
                DualNum6 r2 = q2 / p2;

                if (p1 < 0)
                {
                    if (r1 > maxNegative)
                    {
                        maxNegative = r1;
                    }

                    if (r2 < minPositive)
                    {
                        minPositive = r2;
                    }
                }
                else
                {
                    if (r2 > maxNegative)
                    {
                        maxNegative = r2;
                    }

                    if (r1 < minPositive)
                    {
                        minPositive = r1;
                    }
                }
            }

            if (!p3_zero)
            {
                DualNum6 r3 = q3 / p3;
                DualNum6 r4 = q4 / p4;

                if (p3 < 0)
                {
                    if (r3 > maxNegative)
                    {
                        maxNegative = r3;
                    }

                    if (r4 < minPositive)
                    {
                        minPositive = r4;
                    }
                }
                else
                {
                    if (r4 > maxNegative)
                    {
                        maxNegative = r4;
                    }

                    if (r3 < minPositive)
                    {
                        minPositive = r3;
                    }
                }
            }

            if (!p5_zero)
            {
                DualNum6 r5 = q5 / p5;
                DualNum6 r6 = q6 / p6;

                if (p5 < 0)
                {
                    if (r5 > maxNegative)
                    {
                        maxNegative = r5;
                    }

                    if (r6 < minPositive)
                    {
                        minPositive = r6;
                    }
                }
                else
                {
                    if (r6 > maxNegative)
                    {
                        maxNegative = r6;
                    }

                    if (r5 < minPositive)
                    {
                        minPositive = r5;
                    }
                }
            }

            if (maxNegative > minPositive)
            {
                result = maxNegative; // assign garbage to appease compiler
                return false;
            }

            // Center point of the intersecting portion of the line segment
            var centerParam = (minPositive + maxNegative) * 0.5;
            var centerX = x1 + p2 * centerParam;
            var centerY = y1 + p4 * centerParam;
            var centerZ = z1 + p6 * centerParam;

            var minDX = DualNum6.Min(maxBoxBounds.X - centerX, centerX - minBoxBounds.X);
            var minDY = DualNum6.Min(maxBoxBounds.Y - centerY, centerY - minBoxBounds.Y);
            var minDZ = DualNum6.Min(maxBoxBounds.Z - centerZ, centerZ - minBoxBounds.Z);

            // Minimal distance from the center point to the box boundaries
            result = DualNum6.Min(minDX, minDY, minDZ);

            return true;
        }
    }

    // public struct CollisionConstraint
    // {
    //     public FastVec3d collisionBoxPlanes;

    //     public float Friction;
    //     public ushort PointIndex;

    //     public byte planeDirs;

    //     public void Update(Span<ClothPointData> points, TimeStepData time)
    //     {
    //         const double epsilon = 0.00001;

    //         ref var p = ref points[PointIndex];

    //         if (p.Movable == 0) return;

    //         FastVec3d gradient;

    //         (double constraintValue, char moveDir) = CalcCollisionConstraint(in p.Pos, out gradient);

    //         if (constraintValue > 0.0) return;

    //         double delta;
    //         double deltaBorder;

    //         switch (moveDir)
    //         {
    //         case 'X':
    //             delta = p.Pos.X - p.PrevPos.X;
    //             deltaBorder = (p.Pos.X - gradient.X * constraintValue) - p.PrevPos.X;
    //             break;
    //         case 'Y':
    //             delta = p.Pos.Y - p.PrevPos.Y;
    //             deltaBorder = (p.Pos.Y - gradient.Y * constraintValue) - p.PrevPos.Y;
    //             break;
    //         case 'Z':
    //             delta = p.Pos.Z - p.PrevPos.Z;
    //             deltaBorder = (p.Pos.Z - gradient.Z * constraintValue) - p.PrevPos.Z;
    //             break;
    //         default:
    //             throw new System.Diagnostics.UnreachableException();
    //         }

    //         double t = (Math.Abs(delta) > epsilon) ? (deltaBorder / delta) : 0.5;

    //         t = Math.Clamp(t, 0.0, 1.0);

    //         // Point of collision for the line going from previous to current position
    //         // This point is the final result if the friction is maximum (1.0)
    //         FastVec3d collisionPoint;
    //         collisionPoint.X = p.Pos.X * t + p.PrevPos.X * (1.0 - t);
    //         collisionPoint.Y = p.Pos.Y * t + p.PrevPos.Y * (1.0 - t);
    //         collisionPoint.Z = p.Pos.Z * t + p.PrevPos.Z * (1.0 - t);

    //         // Result of extruding current position along collision box normal
    //         // This point is the final result if there is no friction (0.0)
    //         FastVec3d slidePoint;
    //         slidePoint.X = p.Pos.X - gradient.X * constraintValue;
    //         slidePoint.Y = p.Pos.Y - gradient.Y * constraintValue;
    //         slidePoint.Z = p.Pos.Z - gradient.Z * constraintValue;

    //         p.Pos.X = slidePoint.X * (1.0 - Friction) + collisionPoint.X * Friction;
    //         p.Pos.Y = slidePoint.Y * (1.0 - Friction) + collisionPoint.Y * Friction;
    //         p.Pos.Z = slidePoint.Z * (1.0 - Friction) + collisionPoint.Z * Friction;
    //     }

    //     [MethodImpl(MethodImplOptions.AggressiveInlining)]
    //     public (double, char) CalcCollisionConstraint(in FastVec3d pos, out FastVec3d gradient)
    //     {
    //         FastVec3d planeDirections;
    //         planeDirections.X = (double)(2 * (sbyte)((planeDirs) & 1) - 1);
    //         planeDirections.Y = (double)(2 * (sbyte)((planeDirs >> 1) & 1) - 1);
    //         planeDirections.Z = (double)(2 * (sbyte)((planeDirs >> 2) & 1) - 1);

    //         FastVec3d delta;
    //         delta.X = (pos.X - collisionBoxPlanes.X);
    //         delta.Y = (pos.Y - collisionBoxPlanes.Y);
    //         delta.Z = (pos.Z - collisionBoxPlanes.Z);

    //         FastVec3d dir_delta;
    //         dir_delta.X = delta.X * planeDirections.X;
    //         dir_delta.Y = delta.Y * planeDirections.Y;
    //         dir_delta.Z = delta.Z * planeDirections.Z;

    //         if ((dir_delta.X > dir_delta.Y) && (dir_delta.X > dir_delta.Z))
    //         {
    //             gradient.X = planeDirections.X;
    //             gradient.Y = 0;
    //             gradient.Z = 0;

    //             return (dir_delta.X, 'X');
    //         }
    //         else if ((dir_delta.Y > dir_delta.X) && (dir_delta.Y > dir_delta.Z))
    //         {
    //             gradient.X = 0;
    //             gradient.Y = planeDirections.Y;
    //             gradient.Z = 0;

    //             return (dir_delta.Y, 'Y');
    //         }
    //         else
    //         {
    //             gradient.X = 0;
    //             gradient.Y = 0;
    //             gradient.Z = planeDirections.Z;

    //             return (dir_delta.Z, 'Z');
    //         }
    //     }
    // }


    public struct LengthConstraint
    {
        public float TargetLength;
        public float Compliance;
        public float Damping;
        public ConstraintComparison Comparison;

        public void Update(ClothPointData[] points, ushort bufferHead, ushort bufferSize, ushort bufferMaxSize, TimeStepData time, FastVec3f[] forces)
        {
            if (bufferSize <= 1) return;

            double alpha = (double)Compliance * time.InvSubstepTimeSqr;
            double gamma_u = alpha * (double)Damping;

            // TODO: it would be really great to get rid of this allocation.
            // actually, if would be great to get rid of this whole class
            // and somehow reuse calculations from DistanceConstraint
            var gradients = new FastVec3d[2 * (bufferSize - 1)];
            double sumLengthSqr = 0.0;
            double sumInvMass = 0.0;
            double dampingSum = 0.0;
            byte sumMovable = 0;

            for (int i = 0; i < bufferSize - 1; i++)
            {
                ushort index1 = (ushort)((bufferHead + i) % bufferMaxSize);
                ushort index2 = (ushort)((bufferHead + i + 1) % bufferMaxSize);

                ref var p1 = ref points[index1];
                ref var p2 = ref points[index2];

                int gradient1Idx = i * 2;
                int gradient2Idx = i * 2 + 1;

                sumLengthSqr += CalcDistanceConstraint(in p1.Pos,
                                                       in p2.Pos,
                                                       out gradients[gradient1Idx],
                                                       out gradients[gradient2Idx]);

                double effectiveInvMassP1 = (double)p1.Movable * p1.InvMass;
                double effectiveInvMassP2 = (double)p2.Movable * p2.InvMass;

                sumInvMass += effectiveInvMassP1 * gradients[gradient1Idx].LengthSq();
                sumInvMass += effectiveInvMassP2 * gradients[gradient2Idx].LengthSq();

                sumMovable |= p1.Movable;
                sumMovable |= p2.Movable;

                dampingSum +=
                    (p1.Pos.X - p1.PrevPos.X) * gradients[gradient1Idx].X +
                    (p1.Pos.Y - p1.PrevPos.Y) * gradients[gradient1Idx].Y +
                    (p1.Pos.Z - p1.PrevPos.Z) * gradients[gradient1Idx].Z +

                    (p2.Pos.X - p2.PrevPos.X) * gradients[gradient2Idx].X +
                    (p2.Pos.Y - p2.PrevPos.Y) * gradients[gradient2Idx].Y +
                    (p2.Pos.Z - p2.PrevPos.Z) * gradients[gradient2Idx].Z;
            }

            dampingSum *= gamma_u;

            double constraintValue = sumLengthSqr - TargetLength * TargetLength;

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

            for (int i = 0; i < bufferSize - 1; i--)
            {
                ushort index1 = (ushort)((bufferHead + i) % bufferMaxSize);
                ushort index2 = (ushort)((bufferHead + i + 1) % bufferMaxSize);

                ref var p1 = ref points[index1];
                ref var p2 = ref points[index2];

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

                if (forces != null)
                {
                    double invGradientP1 = 1 / gradientP1.Length();
                    double invGradientP2 = 1 / gradientP2.Length();

                    forces[index1].X += (float)(gradientP1.X * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                    forces[index1].Y += (float)(gradientP1.Y * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                    forces[index1].Z += (float)(gradientP1.Z * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));

                    forces[index2].X += (float)(gradientP2.X * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                    forces[index2].Y += (float)(gradientP2.Y * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                    forces[index2].Z += (float)(gradientP2.Z * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double CalcDistanceConstraint(in FastVec3d pos1,
                                             in FastVec3d pos2,
                                             out FastVec3d gradient1,
                                             out FastVec3d gradient2)
        {
            // Distance between two points:
            //
            // C = |p1 - p2| - target_dist
            //
            FastVec3d diff;
            diff.X = pos1.X - pos2.X;
            diff.Y = pos1.Y - pos2.Y;
            diff.Z = pos1.Z - pos2.Z;

            double lengthSqr = diff.LengthSq();

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            gradient1.X = 2 * diff.X;
            gradient1.Y = 2 * diff.Y;
            gradient1.Z = 2 * diff.Z;

            // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
            gradient2.X = -gradient1.X;
            gradient2.Y = -gradient1.Y;
            gradient2.Z = -gradient1.Z;

            return lengthSqr;
        }
    }

    [ProtoContract]
    public struct DistanceConstraint
    {
        [ProtoMember(4)]
        public byte PointIndex1;
        [ProtoMember(5)]
        public byte PointIndex2;
        [ProtoMember(6)]
        public ConstraintComparison Comparison;

        public void Update(ClothPointData[] points,
                           TimeStepData time,
                           float TargetDistance,
                           float Compliance,
                           float Damping,
                           FastVec3f[] forces = null)
        {
            ref var p1 = ref points[PointIndex1];
            ref var p2 = ref points[PointIndex2];

            if ((p1.Movable == 0) && (p2.Movable == 0)) return;

            FastVec3d gradientP1;
            FastVec3d gradientP2;

            double constraintValue = CalcDistanceConstraint(in p1.Pos, in p2.Pos,
                                                            out gradientP1, out gradientP2,
                                                            TargetDistance);

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
                ( alpha +
                  (1.0 + gamma_u * time.InvSubstepTime) *
                  (gradientP1.LengthSq() * effectiveInvMassP1 + gradientP2.LengthSq() * effectiveInvMassP2) );

            // LagrangeMultiplier += lambda_delta;

            // Since we are using Verlet integration, simply updating position here
            // also implicitly updates the velocity of the points

            p1.Pos.X += gradientP1.X * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Y += gradientP1.Y * (lambda_delta * effectiveInvMassP1);
            p1.Pos.Z += gradientP1.Z * (lambda_delta * effectiveInvMassP1);

            p2.Pos.X += gradientP2.X * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Y += gradientP2.Y * (lambda_delta * effectiveInvMassP2);
            p2.Pos.Z += gradientP2.Z * (lambda_delta * effectiveInvMassP2);

            if (forces != null)
            {
                double invGradientP1 = 1 / gradientP1.Length();
                double invGradientP2 = 1 / gradientP2.Length();

                forces[PointIndex1].X += (float)(gradientP1.X * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Y += (float)(gradientP1.Y * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Z += (float)(gradientP1.Z * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));

                forces[PointIndex2].X += (float)(gradientP2.X * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Y += (float)(gradientP2.Y * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Z += (float)(gradientP2.Z * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double CalcDistanceConstraint(in FastVec3d pos1,
                                             in FastVec3d pos2,
                                             out FastVec3d gradient1,
                                             out FastVec3d gradient2,
                                             float TargetDistance)
        {
            // Distance between two points:
            //
            // C = |p1 - p2| - target_dist
            //
            FastVec3d diff;
            diff.X = pos1.X - pos2.X;
            diff.Y = pos1.Y - pos2.Y;
            diff.Z = pos1.Z - pos2.Z;

            double lengthSqr = diff.LengthSq();

            // Derivative of distance between two points,
            // with respect to each of the points:
            //
            // C' = |p1 - p2|' = (p1 - p2)' * (p1 - p2) / |p1 - p2|

            // C'_p1 = (1 - 0) * (p1 - p2) / |p1 - p2| = (p1 - p2) / |p1 - p2|
            gradient1.X = 2 * diff.X;
            gradient1.Y = 2 * diff.Y;
            gradient1.Z = 2 * diff.Z;

            // C'_p1 = (0 - 1) * (p1 - p2) / |p1 - p2| = - (p1 - p2) / |p1 - p2|
            gradient2.X = -gradient1.X;
            gradient2.Y = -gradient1.Y;
            gradient2.Z = -gradient1.Z;

            return lengthSqr - (double)TargetDistance * TargetDistance;
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

        public void Update(ClothPointData[] points, TimeStepData time, FastVec3f[] forces)
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

            if (forces != null)
            {
                double invGradientP1 = 1 / gradientP1.Length();
                double invGradientP2 = 1 / gradientP2.Length();
                double invGradientP3 = 1 / gradientP3.Length();

                forces[PointIndex1].X += (float)(gradientP1.X * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Y += (float)(gradientP1.Y * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex1].Z += (float)(gradientP1.Z * (invGradientP1 * time.InvSubstepTimeSqr * lambda_delta));

                forces[PointIndex2].X += (float)(gradientP2.X * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Y += (float)(gradientP2.Y * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex2].Z += (float)(gradientP2.Z * (invGradientP2 * time.InvSubstepTimeSqr * lambda_delta));

                forces[PointIndex3].X += (float)(gradientP3.X * (invGradientP3 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex3].Y += (float)(gradientP3.Y * (invGradientP3 * time.InvSubstepTimeSqr * lambda_delta));
                forces[PointIndex3].Z += (float)(gradientP3.Z * (invGradientP3 * time.InvSubstepTimeSqr * lambda_delta));
            }
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
