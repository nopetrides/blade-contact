using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// Registers authored blade-contact geometry on a dynamic sword. The profile is supplied by the
    /// consumer; the package contains no specimen dimensions and no study-declared sharpness criterion.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class BladeShell : MonoBehaviour
    {
        [Tooltip("The dynamic body this shell belongs to. The solver returns impulses to it; it never " +
                 "takes pose authority over it.")]
        [SerializeField] private Rigidbody body;

        [Tooltip("Authored cross-section geometry for this specimen, in this transform's local space.")]
        [SerializeField] private BladeProfileAsset profileAsset;

        [Tooltip("The PhysX collider representing THIS BLADE ONLY. When two shells are registered as a " +
                 "pair, the manager suppresses PhysX collision between exactly these two colliders, so " +
                 "one owner reports blade-vs-blade. Guard, grip, pommel and world collision are untouched.")]
        [SerializeField] private Collider bladeCollider;

        private BladeShellData cachedData;

        /// <summary>The dynamic body receiving custom sword-to-sword response.</summary>
        public Rigidbody Body => body;

        /// <summary>Built surfaces and edge lines for this shell, constructed once and reused.</summary>
        public BladeShellData Data =>
            cachedData ??= profileAsset != null
                ? BladeShellBuilder.Build(profileAsset.Profile)
                : new BladeShellData(new BladeSurface[0], new BladeEdgeLine[0]);

        /// <summary>
        /// The blade's own PhysX collider, the single relationship handed over to the custom solver.
        /// </summary>
        /// <remarks>
        /// Named explicitly rather than discovered by convention. Suppressing the wrong collider would
        /// silently drop guard or grip collision, which is a change to the physical model rather than to
        /// who owns blade contact, and it would not be obvious from watching the scene.
        /// </remarks>
        public Collider BladeCollider => bladeCollider;

        /// <summary>Current world pose of this shell's local geometry.</summary>
        public BladePose CurrentPose => new BladePose(transform.position, transform.rotation);

        /// <summary>Reusable world-space buffers, so a per-step sweep does not allocate.</summary>
        public BladeShellScratch Scratch { get; } = new BladeShellScratch();

        /// <summary>Discards the built shell after the authored profile changes.</summary>
        public void InvalidateData() => cachedData = null;

        /// <summary>
        /// Authoring entry point for consumer tooling that generates prefabs.
        /// </summary>
        /// <remarks>
        /// A profile is specimen-specific: it carries that specimen's own edge basis and feature
        /// identities, so passing another specimen's profile is a modelling error, not a shortcut.
        /// </remarks>
        public void SetAuthoring(BladeProfileAsset profile, Rigidbody dynamicBody, Collider blade = null)
        {
            profileAsset = profile;
            body = dynamicBody;
            if (blade != null) bladeCollider = blade;
            cachedData = null;
        }

        /// <summary>The profile currently driving this shell, or null when none has been assigned.</summary>
        public BladeProfileAsset ProfileAsset => profileAsset;

        private void Reset()
        {
            body = GetComponentInParent<Rigidbody>();
        }

        private void OnValidate()
        {
            if (body == null)
                body = GetComponentInParent<Rigidbody>();

            cachedData = null;
        }
    }
}
