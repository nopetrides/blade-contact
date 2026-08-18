using UnityEngine;

namespace BladeContact
{
    /// <summary>Project asset wrapper so a consumer can author and version a profile outside code.</summary>
    /// <remarks>
    /// A profile is specimen-specific. It carries that specimen's own edge basis and its own feature
    /// identities, so one specimen's profile must never be assigned to another.
    /// </remarks>
    [CreateAssetMenu(menuName = "Blade Contact/Blade Profile", fileName = "BladeProfile")]
    public sealed class BladeProfileAsset : ScriptableObject
    {
        [SerializeField] private BladeProfile profile = new BladeProfile();

        [Header("Authored basis (consumer's convention, expressed in shell-local space)")]
        [Tooltip("Direction along the blade, from guard toward tip.")]
        [SerializeField] private Vector3 bladeAxis = new Vector3(-1f, 0f, 0f);

        [Tooltip("Signed direction across the blade's width, toward the +edge. The opposite edge is -this.")]
        [SerializeField] private Vector3 edgeAxis = new Vector3(0f, 0f, 1f);

        [Tooltip("Normal of the blade's flat, completing the basis.")]
        [SerializeField] private Vector3 faceNormal = new Vector3(0f, 1f, 0f);

        [Tooltip("Where this basis and these features came from, and what is provisional about them.")]
        [TextArea(4, 20)]
        [SerializeField] private string provenance = string.Empty;

        public BladeProfile Profile => profile;

        /// <summary>Direction along the blade, guard toward tip, in shell-local space.</summary>
        public Vector3 BladeAxis => bladeAxis;

        /// <summary>
        /// Signed across-width direction toward the +edge. The package never infers this; it is supplied so a
        /// consumer's own edge convention and the solver's geometry cannot silently disagree in orientation.
        /// </summary>
        public Vector3 EdgeAxis => edgeAxis;

        public Vector3 FaceNormal => faceNormal;

        /// <summary>Human-readable record of where the basis and feature identities came from.</summary>
        public string Provenance => provenance;

        /// <summary>Authoring entry point for consumer-side tooling that generates a profile.</summary>
        public void SetAuthored(
            BladeProfile authoredProfile, Vector3 axis, Vector3 edge, Vector3 face, string provenanceNotes)
        {
            profile = authoredProfile;
            bladeAxis = axis;
            edgeAxis = edge;
            faceNormal = face;
            provenance = provenanceNotes;
        }
    }
}
