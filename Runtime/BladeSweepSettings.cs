using System;
using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// Numerical settings for conservative advancement. These are implementation tolerances, not
    /// physical claims: they are exposed for review and chosen conservatively. The consumer supplies
    /// study-declared physical criteria (sharpness, friction, bind rule) elsewhere.
    /// </summary>
    [Serializable]
    public struct BladeSweepSettings
    {
        [SerializeField] private float contactMargin;
        [SerializeField] private int maxIterations;
        [SerializeField] private float minimumTimeAdvance;
        [SerializeField] private float minimumClosureRate;

        public BladeSweepSettings(
            float contactMargin, int maxIterations, float minimumTimeAdvance, float minimumClosureRate)
        {
            this.contactMargin = contactMargin;
            this.maxIterations = maxIterations;
            this.minimumTimeAdvance = minimumTimeAdvance;
            this.minimumClosureRate = minimumClosureRate;
        }

        /// <summary>
        /// Surface separation at which contact is declared. Advancement stops here rather than at zero
        /// so the returned pose is strictly non-penetrating.
        /// </summary>
        public float ContactMargin => contactMargin;

        /// <summary>
        /// Iteration ceiling. Exhausting it yields <see cref="BladeSweepStatus.IterationLimit"/>, which
        /// still blocks motion: the sweep fails toward non-crossing, never toward tunnelling.
        /// </summary>
        public int MaxIterations => maxIterations;

        /// <summary>
        /// Advancement converges on contact geometrically, so the separation test alone is reached only
        /// asymptotically. A step smaller than this counts as converged and declares contact, which
        /// stops short of the true contact time and therefore stays conservative.
        /// </summary>
        public float MinimumTimeAdvance => minimumTimeAdvance;

        /// <summary>Closure-rate bound below which the shells cannot approach and the sweep exits.</summary>
        public float MinimumClosureRate => minimumClosureRate;

        public static BladeSweepSettings Default =>
            new BladeSweepSettings(0.0005f, 64, 1e-6f, 1e-6f);

        /// <summary>Returns these settings with a different contact margin, for tolerance checks.</summary>
        public BladeSweepSettings WithContactMargin(float margin) =>
            new BladeSweepSettings(margin, maxIterations, minimumTimeAdvance, minimumClosureRate);
    }
}
