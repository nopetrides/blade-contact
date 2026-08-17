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

        private BladeShellData cachedData;

        /// <summary>The dynamic body receiving custom sword-to-sword response.</summary>
        public Rigidbody Body => body;

        /// <summary>Built surfaces and edge lines for this shell, constructed once and reused.</summary>
        public BladeShellData Data =>
            cachedData ??= profileAsset != null
                ? BladeShellBuilder.Build(profileAsset.Profile)
                : new BladeShellData(new BladeSurface[0], new BladeEdgeLine[0]);

        /// <summary>Current world pose of this shell's local geometry.</summary>
        public BladePose CurrentPose => new BladePose(transform.position, transform.rotation);

        /// <summary>Reusable world-space buffers, so a per-step sweep does not allocate.</summary>
        public BladeShellScratch Scratch { get; } = new BladeShellScratch();

        /// <summary>Discards the built shell after the authored profile changes.</summary>
        public void InvalidateData() => cachedData = null;

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
