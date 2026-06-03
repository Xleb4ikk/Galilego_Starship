using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using Galilego.Universe;

namespace Galilego.UI
{
    public sealed class ReferenceFrameSelectorUI : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private TMP_Dropdown frameDropdown;
        [SerializeField] private Toggle autoSphereOfInfluenceToggle;
        [SerializeField] private Button cycleButton;
        [SerializeField] private TMP_Text activeFrameLabel;

        private readonly List<ReferenceFrameTarget> dropdownTargets = new List<ReferenceFrameTarget>();
        private bool isUpdatingUi;

        private void Awake()
        {
            ResolveReferences();
            PopulateDropdown();
        }

        private void OnEnable()
        {
            ResolveReferences();
            PopulateDropdown();
            Subscribe();
            RefreshUi();
        }

        private void OnDisable()
        {
            Unsubscribe();
        }

        private void Update()
        {
            RefreshUi();
        }

        private void Subscribe()
        {
            if (frameDropdown != null)
            {
                frameDropdown.onValueChanged.RemoveListener(HandleDropdownChanged);
                frameDropdown.onValueChanged.AddListener(HandleDropdownChanged);
            }

            if (autoSphereOfInfluenceToggle != null)
            {
                autoSphereOfInfluenceToggle.onValueChanged.RemoveListener(HandleAutoToggleChanged);
                autoSphereOfInfluenceToggle.onValueChanged.AddListener(HandleAutoToggleChanged);
            }

            if (cycleButton != null)
            {
                cycleButton.onClick.RemoveListener(HandleCycleClicked);
                cycleButton.onClick.AddListener(HandleCycleClicked);
            }

            if (universeManager != null)
            {
                universeManager.ActiveReferenceFrameChanged -= HandleActiveFrameChanged;
                universeManager.ActiveReferenceFrameChanged += HandleActiveFrameChanged;
            }
        }

        private void Unsubscribe()
        {
            if (frameDropdown != null)
            {
                frameDropdown.onValueChanged.RemoveListener(HandleDropdownChanged);
            }

            if (autoSphereOfInfluenceToggle != null)
            {
                autoSphereOfInfluenceToggle.onValueChanged.RemoveListener(HandleAutoToggleChanged);
            }

            if (cycleButton != null)
            {
                cycleButton.onClick.RemoveListener(HandleCycleClicked);
            }

            if (universeManager != null)
            {
                universeManager.ActiveReferenceFrameChanged -= HandleActiveFrameChanged;
            }
        }

        private void PopulateDropdown()
        {
            if (frameDropdown == null)
            {
                return;
            }

            dropdownTargets.Clear();
            frameDropdown.ClearOptions();

            List<TMP_Dropdown.OptionData> options = new List<TMP_Dropdown.OptionData>();
            foreach (ReferenceFrameTarget target in Enum.GetValues(typeof(ReferenceFrameTarget)))
            {
                dropdownTargets.Add(target);
                options.Add(new TMP_Dropdown.OptionData(target.ToString()));
            }

            frameDropdown.AddOptions(options);
        }

        private void RefreshUi()
        {
            if (universeManager == null)
            {
                ResolveReferences();
                if (universeManager == null)
                {
                    return;
                }
            }

            isUpdatingUi = true;

            if (frameDropdown != null)
            {
                int selectedIndex = dropdownTargets.IndexOf(universeManager.SelectedReferenceFrame);
                if (selectedIndex >= 0 && frameDropdown.value != selectedIndex)
                {
                    frameDropdown.SetValueWithoutNotify(selectedIndex);
                }
            }

            if (autoSphereOfInfluenceToggle != null)
            {
                autoSphereOfInfluenceToggle.SetIsOnWithoutNotify(universeManager.IsAutoSphereOfInfluenceSelectionEnabled);
            }

            if (activeFrameLabel != null)
            {
                ReferenceFrameTarget activeFrame = universeManager.ActiveReferenceFrame;
                activeFrameLabel.text = universeManager.IsAutoSphereOfInfluenceSelectionEnabled
                    ? $"Frame: {activeFrame} (Auto SOI)"
                    : $"Frame: {activeFrame}";
            }

            isUpdatingUi = false;
        }

        private void HandleDropdownChanged(int optionIndex)
        {
            if (isUpdatingUi || universeManager == null || optionIndex < 0 || optionIndex >= dropdownTargets.Count)
            {
                return;
            }

            universeManager.SelectReferenceFrame(dropdownTargets[optionIndex]);
            RefreshUi();
        }

        private void HandleAutoToggleChanged(bool enabled)
        {
            if (isUpdatingUi || universeManager == null)
            {
                return;
            }

            universeManager.SetAutoSphereOfInfluenceSelection(enabled);
            RefreshUi();
        }

        private void HandleCycleClicked()
        {
            if (universeManager == null)
            {
                return;
            }

            universeManager.CycleReferenceFrame();
            RefreshUi();
        }

        private void HandleActiveFrameChanged(ReferenceFrameTarget target)
        {
            RefreshUi();
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
            {
                universeManager = FindAnyObjectByType<UniverseManager>();
            }
        }
    }
}
