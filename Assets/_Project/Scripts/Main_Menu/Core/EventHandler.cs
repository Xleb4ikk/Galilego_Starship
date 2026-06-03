using System;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class EventHandler : MonoBehaviour, IPointerEnterHandler,
    IPointerExitHandler,
    IPointerClickHandler
{
    public Action<EventHandler> OnClick;
    public Action<EventHandler> OnHoverEnter;
    public Action<EventHandler> OnHoverExit;
    public Action<EventHandler, int> OnDropdownValueChanged;
    public Action<EventHandler, float> OnSliderValueChanged;

    private void Awake()
    {
        var dropdown = GetComponent<TMP_Dropdown>();
        if (dropdown != null)
        {
            dropdown.onValueChanged.AddListener(DropdownChanged);
        }

        var legacyDropdown = GetComponent<Dropdown>();
        if (legacyDropdown != null)
        {
            legacyDropdown.onValueChanged.AddListener(DropdownChanged);
        }

        var slider = GetComponent<Slider>();
        if (slider != null)
        {
            Debug.Log("Slider found: " + slider);
            slider.onValueChanged.AddListener(OnSliderChanged);
        }
    }

    void Start()
    {
        DebugManager.Log($"Start EventHandler: {this}");
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        OnHoverEnter?.Invoke(this);
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        OnHoverExit?.Invoke(this);
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        OnClick?.Invoke(this);
    }

    private void DropdownChanged(int value)
    {
        OnDropdownValueChanged?.Invoke(this, value);
    }

    private void OnSliderChanged(float value)
    {
        OnSliderValueChanged?.Invoke(this, value);
    }
}
