using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// Reusable world-space buffers for one shell, so a per-step sweep does not allocate.
    /// </summary>
    public sealed class BladeShellScratch
    {
        internal Vector3[] SurfaceCentres = new Vector3[0];
        internal Vector3[] EdgeCentres = new Vector3[0];
        internal Vector3[] GroupCentres = new Vector3[0];
        internal int[] PairOrder = new int[0];
        internal float[] PairBound = new float[0];

        /// <summary>Group centres only. Cheap enough to run at every iterate of the sweep.</summary>
        internal void PrepareGroups(BladeShellData shell, in BladePose pose)
        {
            if (GroupCentres.Length < shell.GroupCount)
                GroupCentres = new Vector3[shell.GroupCount];

            for (int i = 0; i < shell.GroupCount; i++)
                GroupCentres[i] = pose.TransformPoint(shell.GetGroup(i).LocalCentre);
        }

        /// <summary>Per-feature centres. Only needed when the sweep refines to exact measurement.</summary>
        internal void PrepareFeatures(BladeShellData shell, in BladePose pose)
        {
            if (SurfaceCentres.Length < shell.SurfaceCount)
                SurfaceCentres = new Vector3[shell.SurfaceCount];
            if (EdgeCentres.Length < shell.EdgeCount)
                EdgeCentres = new Vector3[shell.EdgeCount];

            for (int i = 0; i < shell.SurfaceCount; i++)
                SurfaceCentres[i] = pose.TransformPoint(shell.SurfaceCentre(i));
            for (int i = 0; i < shell.EdgeCount; i++)
                EdgeCentres[i] = pose.TransformPoint(shell.EdgeCentre(i));
        }

        internal void EnsurePairCapacity(int count)
        {
            if (PairOrder.Length < count)
            {
                PairOrder = new int[count];
                PairBound = new float[count];
            }
        }
    }

    /// <summary>
    /// Continuous first-contact search between two authored blade shells, by conservative advancement
    /// over their real surface and edge pieces.
    /// </summary>
    /// <remarks>
    /// Unity's sweep-based continuous collision detection cannot perform angular sweeps, so a shell that
    /// rotates through another between two simulation steps can exchange sides while both endpoint poses
    /// are separated. Conservative advancement is angular-capable: each iterate measures the true
    /// separation between authored surfaces and advances time by only what an upper bound on the closure
    /// rate permits, so no contact earlier than the returned time of impact can exist along the path.
    ///
    /// Nothing here is inflated. Distances are between the authored triangles and edge lines themselves,
    /// so a blade's thickness comes from its opposing faces converging at the designated edge rather than
    /// from a radius swept along a centre line.
    ///
    /// This type reads poses and returns a time of impact. It never writes a pose back onto a body:
    /// registered swords remain dynamic Rigidbodies owned by Unity.
    /// </remarks>
    public static class BladeShellSweep
    {
        /// <summary>
        /// Separation band within which two candidate feature pairs count as tied. An edge line lies on
        /// the boundary of its adjoining surfaces, so the two genuinely coincide; the tie is broken toward
        /// the edge because it is the more specific description of the same contact.
        /// </summary>
        private const float SpecificityTieBand = 1e-6f;

        public static BladeShellContact FindFirstContact(
            BladeShellData shellA, in BladePose startA, in BladePose endA,
            BladeShellData shellB, in BladePose startB, in BladePose endB,
            in BladeSweepSettings settings,
            BladeShellScratch scratchA = null, BladeShellScratch scratchB = null)
        {
            scratchA = scratchA ?? new BladeShellScratch();
            scratchB = scratchB ?? new BladeShellScratch();

            float angleA = BladePose.AngleRadians(startA, endA);
            float angleB = BladePose.AngleRadians(startB, endB);
            Vector3 relativeTranslation = (endA.Position - startA.Position) - (endB.Position - startB.Position);

            float closureBound =
                relativeTranslation.magnitude +
                angleA * shellA.LocalExtent +
                angleB * shellB.LocalExtent;

            float time = 0f;
            BladeFeaturePair pair = BladeFeaturePair.None;

            for (int iteration = 1; iteration <= settings.MaxIterations; iteration++)
            {
                BladePose poseA = BladePose.Interpolate(startA, endA, time);
                BladePose poseB = BladePose.Interpolate(startB, endB, time);

                if (closureBound <= settings.MinimumClosureRate)
                    return new BladeShellContact(BladeContactStatus.NoContact, 1f, pair, iteration);

                // While the shells are still far apart, a bounding-volume lower bound is enough to take
                // a safe step, and costs a few hundred sphere tests instead of tens of thousands of
                // triangle tests. Understating the separation only shortens the step; it can never step
                // over a contact. Contact is still declared from the exact measurement below.
                scratchA.PrepareGroups(shellA, poseA);
                scratchB.PrepareGroups(shellB, poseB);
                float coarse = CoarseLowerBound(shellA, scratchA, shellB, scratchB);

                if (coarse > settings.CoarseRefinementBand)
                {
                    time += (coarse - settings.ContactMargin) / closureBound;
                    if (time >= 1f)
                        return new BladeShellContact(BladeContactStatus.NoContact, 1f, pair, iteration);
                    continue;
                }

                pair = ClosestFeaturePair(shellA, poseA, shellB, poseB, scratchA, scratchB);

                // The shells cannot close faster than closureBound, so advancing by the separation
                // divided by that bound can never step over a contact.
                float advance = (pair.Separation - settings.ContactMargin) / closureBound;

                if (pair.Separation <= settings.ContactMargin || advance < settings.MinimumTimeAdvance)
                    return new BladeShellContact(BladeContactStatus.Contact, time, pair, iteration);

                time += advance;

                if (time >= 1f)
                    return new BladeShellContact(BladeContactStatus.NoContact, 1f, pair, iteration);
            }

            // Did not converge. Block the motion at the last proven-safe time rather than accept a step
            // that could cross.
            return new BladeShellContact(BladeContactStatus.IterationLimit, time, pair, settings.MaxIterations);
        }

        /// <summary>
        /// Closest authored feature pair between two shells at fixed poses.
        /// </summary>
        /// <remarks>
        /// Reports separation, not penetration depth: intersecting geometry returns zero rather than a
        /// negative number. Conservative advancement never needs a depth, because it stops short of
        /// contact by construction.
        /// </remarks>
        public static BladeFeaturePair ClosestFeaturePair(
            BladeShellData shellA, in BladePose poseA,
            BladeShellData shellB, in BladePose poseB,
            BladeShellScratch scratchA = null, BladeShellScratch scratchB = null)
        {
            scratchA = scratchA ?? new BladeShellScratch();
            scratchB = scratchB ?? new BladeShellScratch();
            scratchA.PrepareGroups(shellA, poseA);
            scratchB.PrepareGroups(shellB, poseB);
            scratchA.PrepareFeatures(shellA, poseA);
            scratchB.PrepareFeatures(shellB, poseB);

            int groupCountB = shellB.GroupCount;
            int groupPairs = shellA.GroupCount * groupCountB;
            scratchA.EnsurePairCapacity(groupPairs);

            // Cheap lower bound per group pair, remembering the single most promising one. Deliberately
            // not a sort: at fine tessellation there are thousands of group pairs, and ordering them all
            // costs far more than the triangle work it would save.
            int closestSlot = 0;
            float closestBound = float.MaxValue;

            for (int i = 0; i < shellA.GroupCount; i++)
            {
                BladeShellGroup ga = shellA.GetGroup(i);
                Vector3 ca = scratchA.GroupCentres[i];
                for (int j = 0; j < groupCountB; j++)
                {
                    int slot = i * groupCountB + j;
                    float bound = (ca - scratchB.GroupCentres[j]).magnitude
                                  - ga.LocalRadius - shellB.GetGroup(j).LocalRadius;
                    scratchA.PairBound[slot] = bound;
                    if (bound < closestBound)
                    {
                        closestBound = bound;
                        closestSlot = slot;
                    }
                }
            }

            float best = float.MaxValue;
            int bestSpecificity = -1;
            BladeFeaturePair bestPair = BladeFeaturePair.None;

            // Measure the most promising group pair first so the running best is tight straight away;
            // every later pair is then rejected by its bound without touching a triangle.
            EvaluateGroupPair(
                shellA, poseA, scratchA, shellA.GetGroup(closestSlot / groupCountB),
                shellB, poseB, scratchB, shellB.GetGroup(closestSlot % groupCountB),
                ref best, ref bestSpecificity, ref bestPair);

            for (int slot = 0; slot < groupPairs; slot++)
            {
                if (slot == closestSlot || scratchA.PairBound[slot] >= best) continue;

                EvaluateGroupPair(
                    shellA, poseA, scratchA, shellA.GetGroup(slot / groupCountB),
                    shellB, poseB, scratchB, shellB.GetGroup(slot % groupCountB),
                    ref best, ref bestSpecificity, ref bestPair);
            }

            return bestPair;
        }

        /// <summary>
        /// Lower bound on the true separation, from group bounding spheres alone. Requires
        /// <see cref="BladeShellScratch.PrepareGroups"/> to have run for both shells.
        /// </summary>
        private static float CoarseLowerBound(
            BladeShellData shellA, BladeShellScratch scratchA,
            BladeShellData shellB, BladeShellScratch scratchB)
        {
            float best = float.MaxValue;
            for (int i = 0; i < shellA.GroupCount; i++)
            {
                BladeShellGroup ga = shellA.GetGroup(i);
                Vector3 ca = scratchA.GroupCentres[i];
                for (int j = 0; j < shellB.GroupCount; j++)
                {
                    float d = (ca - scratchB.GroupCentres[j]).magnitude - ga.LocalRadius - shellB.GetGroup(j).LocalRadius;
                    if (d < best) best = d;
                }
            }

            return best;
        }

        private static void EvaluateGroupPair(
            BladeShellData shellA, in BladePose poseA, BladeShellScratch scratchA, in BladeShellGroup ga,
            BladeShellData shellB, in BladePose poseB, BladeShellScratch scratchB, in BladeShellGroup gb,
            ref float best, ref int bestSpecificity, ref BladeFeaturePair bestPair)
        {
            int aSurfaceEnd = ga.SurfaceStart + ga.SurfaceCount;
            int bSurfaceEnd = gb.SurfaceStart + gb.SurfaceCount;
            int aEdgeEnd = ga.EdgeStart + ga.EdgeCount;
            int bEdgeEnd = gb.EdgeStart + gb.EdgeCount;

            for (int i = ga.SurfaceStart; i < aSurfaceEnd; i++)
            {
                BladeSurface sa = shellA.GetSurface(i);
                Vector3 ca = scratchA.SurfaceCentres[i];
                float ra = shellA.SurfaceRadius(i);
                Vector3 a0 = poseA.TransformPoint(sa.LocalA);
                Vector3 a1 = poseA.TransformPoint(sa.LocalB);
                Vector3 a2 = poseA.TransformPoint(sa.LocalC);

                for (int j = gb.SurfaceStart; j < bSurfaceEnd; j++)
                {
                    if ((ca - scratchB.SurfaceCentres[j]).magnitude - ra - shellB.SurfaceRadius(j) >= best)
                        continue;

                    BladeSurface sb = shellB.GetSurface(j);
                    Vector3 wa, wb;
                    float d = TriangleGeometry.TriangleTriangleDistance(
                        a0, a1, a2,
                        poseB.TransformPoint(sb.LocalA), poseB.TransformPoint(sb.LocalB), poseB.TransformPoint(sb.LocalC),
                        out wa, out wb);

                    Consider(d, 0, new BladeFeatureRef(BladeFeatureKind.Surface, i),
                        new BladeFeatureRef(BladeFeatureKind.Surface, j), wa, wb,
                        ref best, ref bestSpecificity, ref bestPair);
                }

                for (int j = gb.EdgeStart; j < bEdgeEnd; j++)
                {
                    if ((ca - scratchB.EdgeCentres[j]).magnitude - ra - shellB.EdgeRadius(j) >= best)
                        continue;

                    BladeEdgeLine eb = shellB.GetEdge(j);
                    Vector3 wSeg, wTri;
                    float d = TriangleGeometry.SegmentTriangleDistance(
                        poseB.TransformPoint(eb.LocalStart), poseB.TransformPoint(eb.LocalEnd),
                        a0, a1, a2, out wSeg, out wTri);

                    Consider(d, 1, new BladeFeatureRef(BladeFeatureKind.Surface, i),
                        new BladeFeatureRef(BladeFeatureKind.Edge, j), wTri, wSeg,
                        ref best, ref bestSpecificity, ref bestPair);
                }
            }

            for (int i = ga.EdgeStart; i < aEdgeEnd; i++)
            {
                BladeEdgeLine ea = shellA.GetEdge(i);
                Vector3 ca = scratchA.EdgeCentres[i];
                float ra = shellA.EdgeRadius(i);
                Vector3 p = poseA.TransformPoint(ea.LocalStart);
                Vector3 q = poseA.TransformPoint(ea.LocalEnd);

                for (int j = gb.SurfaceStart; j < bSurfaceEnd; j++)
                {
                    if ((ca - scratchB.SurfaceCentres[j]).magnitude - ra - shellB.SurfaceRadius(j) >= best)
                        continue;

                    BladeSurface sb = shellB.GetSurface(j);
                    Vector3 wSeg, wTri;
                    float d = TriangleGeometry.SegmentTriangleDistance(
                        p, q,
                        poseB.TransformPoint(sb.LocalA), poseB.TransformPoint(sb.LocalB), poseB.TransformPoint(sb.LocalC),
                        out wSeg, out wTri);

                    Consider(d, 1, new BladeFeatureRef(BladeFeatureKind.Edge, i),
                        new BladeFeatureRef(BladeFeatureKind.Surface, j), wSeg, wTri,
                        ref best, ref bestSpecificity, ref bestPair);
                }

                for (int j = gb.EdgeStart; j < bEdgeEnd; j++)
                {
                    if ((ca - scratchB.EdgeCentres[j]).magnitude - ra - shellB.EdgeRadius(j) >= best)
                        continue;

                    BladeEdgeLine eb = shellB.GetEdge(j);
                    Vector3 wa, wb;
                    float d = SegmentGeometry.ClosestPointsBetweenSegments(
                        p, q,
                        poseB.TransformPoint(eb.LocalStart), poseB.TransformPoint(eb.LocalEnd),
                        out wa, out wb);

                    Consider(d, 2, new BladeFeatureRef(BladeFeatureKind.Edge, i),
                        new BladeFeatureRef(BladeFeatureKind.Edge, j), wa, wb,
                        ref best, ref bestSpecificity, ref bestPair);
                }
            }
        }

        private static void Consider(
            float separation, int specificity,
            BladeFeatureRef featureA, BladeFeatureRef featureB,
            Vector3 witnessA, Vector3 witnessB,
            ref float best, ref int bestSpecificity, ref BladeFeaturePair bestPair)
        {
            bool closer = separation < best - SpecificityTieBand;
            bool tiedButMoreSpecific = separation < best + SpecificityTieBand && specificity > bestSpecificity;
            if (!closer && !tiedButMoreSpecific) return;

            if (separation < best) best = separation;
            bestSpecificity = specificity;
            bestPair = new BladeFeaturePair(featureA, featureB, witnessA, witnessB, separation);
        }
    }
}
