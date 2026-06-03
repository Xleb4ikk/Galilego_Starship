using UnityEngine;

namespace Galilego.Gameplay
{
    public sealed class BillboardBehaviour : MonoBehaviour
    {
        private Camera mainCamera;

        private void Start()
        {
            mainCamera = Camera.main;
        }

        private void LateUpdate()
        {
            if (mainCamera == null)
            {
                mainCamera = Camera.main;
                if (mainCamera == null) return;
            }

            // Направление К камере (текст должен смотреть НА камеру)
            Vector3 directionToCamera = mainCamera.transform.position - transform.position;
            
            // Поворачиваем объект лицом к камере
            // Используем МИНУС перед directionToCamera, чтобы текст был лицом к камере
            transform.rotation = Quaternion.LookRotation(-directionToCamera, mainCamera.transform.up);
            
        }
    }
}
