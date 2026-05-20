using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

namespace Galilego.UI
{
    public class ExponentialSlider : MonoBehaviour, IDragHandler, IEndDragHandler, IPointerDownHandler
    {
        [Header("Settings")]
        public float Sensitivity = 0.5f;
        public float DeadZone = 2f;
        public float MaxValuePerSecond = 50f;
        public float ExponentialFactor = 2.5f;
        
        [Header("References")]
        [SerializeField] private RectTransform handle;
        [SerializeField] private RectTransform background;

        public event Action<float> OnValueDelta;

        private Vector2 centerPoint;
        private bool isDragging;
        private float currentOffset;

        private void Start()
        {
            if (background != null)
                centerPoint = background.rect.center;
        }

        public void OnPointerDown(PointerEventData eventData)
        {
            isDragging = true;
            OnDrag(eventData);
        }

        public void OnDrag(PointerEventData eventData)
        {
            Vector2 localPoint;
            if (RectTransformUtility.ScreenPointToLocalPointInRectangle(background, eventData.position, eventData.pressEventCamera, out localPoint))
            {
                float offset = localPoint.x - centerPoint.x;
                float halfWidth = background.rect.width * 0.5f;
                currentOffset = Mathf.Clamp(offset, -halfWidth, halfWidth);
                handle.anchoredPosition = new Vector2(currentOffset, handle.anchoredPosition.y);
            }
        }

        public void OnEndDrag(PointerEventData eventData)
        {
            isDragging = false;
            currentOffset = 0f;
            handle.anchoredPosition = new Vector2(0, handle.anchoredPosition.y);
        }

        private void Update()
        {
            if (isDragging && Mathf.Abs(currentOffset) > DeadZone)
            {
                float halfWidth = background.rect.width * 0.5f;
                if (halfWidth <= 0) return;

                float normalizedOffset = currentOffset / halfWidth;
                float sign = Mathf.Sign(normalizedOffset);
                
                // Exponential growth of value change rate
                float rate = Mathf.Pow(Mathf.Abs(normalizedOffset), ExponentialFactor) * sign;
                float val = rate * MaxValuePerSecond * Time.unscaledDeltaTime;
                
                OnValueDelta?.Invoke(val);
            }
        }
    }
}