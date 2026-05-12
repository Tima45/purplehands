#include <openvr_driver.h>
#include <winsock2.h>
#include <vector>
#include <cstring>
#include <cmath>

// --- МАТЕМАТИКА ДЛЯ ШЛЕМА ---
// Извлечение Кватерниона поворота из Матрицы 3x4
vr::HmdQuaternion_t GetMatrixQuat(const vr::HmdMatrix34_t& m) {
    vr::HmdQuaternion_t q;
    q.w = sqrt(fmax(0.0f, 1.0f + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0f;
    q.x = sqrt(fmax(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0f;
    q.y = sqrt(fmax(0.0f, 1.0f - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0f;
    q.z = sqrt(fmax(0.0f, 1.0f - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0f;
    q.x = copysign(q.x, m.m[2][1] - m.m[1][2]);
    q.y = copysign(q.y, m.m[0][2] - m.m[2][0]);
    q.z = copysign(q.z, m.m[1][0] - m.m[0][1]);
    return q;
}

// Перемножение Кватернионов
vr::HmdQuaternion_t MultiplyQuat(const vr::HmdQuaternion_t& q1, const vr::HmdQuaternion_t& q2) {
    vr::HmdQuaternion_t res;
    res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    res.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    res.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return res;
}

// --- БИНАРНАЯ СТРУКТУРА ПАКЕТА ---
// Атрибут packed гарантирует размер ровно 509 байт при компиляции в MinGW
struct __attribute__((packed)) BoneQuat {
    float x, y, z, w;
};

struct __attribute__((packed)) HandPacket {
    uint8_t isRightHand;    
    float posX, posY, posZ; 
    BoneQuat bones[31];     
};

// --- КЛАСС КОНТРОЛЛЕРА ---
class CHandDevice : public vr::ITrackedDeviceServerDriver {
private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;
    vr::VRInputComponentHandle_t m_skeletonHandle;
    bool m_isRightHand;

public:
    CHandDevice(bool isRightHand) : m_isRightHand(isRightHand), m_unObjectId(vr::k_unTrackedDeviceIndexInvalid) {}

    virtual vr::EVRInitError Activate(uint32_t unObjectId) override {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        // Настраиваем свойства под новое имя
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_isRightHand ? "PurpleHand_Right" : "PurpleHand_Left");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5"); 
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, m_isRightHand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        // Пути к иконкам (папка purplehands)
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{purplehands}/icons/hand_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{purplehands}/icons/hand_searching.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{purplehands}/icons/hand_ready.png");

        // Создаем компонент скелета
        const char* skeletonPath = m_isRightHand ? "/input/skeleton/right" : "/input/skeleton/left";
        const char* skeletonBase = m_isRightHand ? "/skeleton/hand/right" : "/skeleton/hand/left";
        
        vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, 
                                                     skeletonPath, skeletonBase, 
                                                     "/pose/raw", 
                                                     vr::VRSkeletalTracking_Partial, 
                                                     nullptr, 0, &m_skeletonHandle);

        return vr::VRInitError_None;
    }

    virtual void Deactivate() override { m_unObjectId = vr::k_unTrackedDeviceIndexInvalid; }
    virtual void EnterStandby() override {}
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override { return nullptr; }
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {}

    virtual vr::DriverPose_t GetPose() override {
        vr::DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
        pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
        pose.qRotation = { 1, 0, 0, 0 }; // Критично для видимости
        return pose;
    }

    void UpdateSkeleton(const HandPacket& packet) {
        if (m_unObjectId == vr::k_unTrackedDeviceIndexInvalid) return;

        // 1. Получаем позу шлема
        vr::TrackedDevicePose_t hmdPose;
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmdPose, 1);
        const vr::HmdMatrix34_t& mat = hmdPose.mDeviceToAbsoluteTracking;

        // 2. Трансформация позиции (локальные от камер -> глобальные в комнате)
        float worldX = mat.m[0][0] * packet.posX + mat.m[0][1] * packet.posY + mat.m[0][2] * packet.posZ + mat.m[0][3];
        float worldY = mat.m[1][0] * packet.posX + mat.m[1][1] * packet.posY + mat.m[1][2] * packet.posZ + mat.m[1][3];
        float worldZ = mat.m[2][0] * packet.posX + mat.m[2][1] * packet.posY + mat.m[2][2] * packet.posZ + mat.m[2][3];

        // 3. Трансформация вращения (поворот шлема * поворот руки)
        vr::HmdQuaternion_t hmdQuat = GetMatrixQuat(mat);
        vr::HmdQuaternion_t handQuat = { packet.bones[0].w, packet.bones[0].x, packet.bones[0].y, packet.bones[0].z };
        vr::HmdQuaternion_t finalQuat = MultiplyQuat(hmdQuat, handQuat);

        // 4. Заполняем кости
        vr::VRBoneTransform_t boneTransforms[31];
        for (int i = 0; i < 31; i++) {
            if (i == 0) {
                boneTransforms[i].position.v[0] = worldX;
                boneTransforms[i].position.v[1] = worldY;
                boneTransforms[i].position.v[2] = worldZ;
            } else {
                boneTransforms[i].position.v[0] = 0.0f;
                boneTransforms[i].position.v[1] = 0.0f;
                boneTransforms[i].position.v[2] = 0.0f;
            }
            boneTransforms[i].orientation.x = finalQuat.x;
            boneTransforms[i].orientation.y = finalQuat.y;
            boneTransforms[i].orientation.z = finalQuat.z;
            boneTransforms[i].orientation.w = finalQuat.w;
        }

        vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithoutController, boneTransforms, 31);

        // Обновляем общую позицию устройства
        vr::DriverPose_t pose = GetPose();
        pose.vecPosition[0] = worldX;
        pose.vecPosition[1] = worldY;
        pose.vecPosition[2] = worldZ;
        pose.qRotation = { finalQuat.w, finalQuat.x, finalQuat.y, finalQuat.z };
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, pose, sizeof(vr::DriverPose_t));
    }
};

// --- МЕНЕДЖЕР ДРАЙВЕРА ---
class CServerDriver_PurpleHands : public vr::IServerTrackedDeviceProvider {
private:
    CHandDevice* m_pLeftHand;
    CHandDevice* m_pRightHand;
    SOCKET m_udpSocket;

public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

        m_pLeftHand = new CHandDevice(false);
        m_pRightHand = new CHandDevice(true);
        vr::VRServerDriverHost()->TrackedDeviceAdded("PurpleHand_Left_001", vr::TrackedDeviceClass_Controller, m_pLeftHand);
        vr::VRServerDriverHost()->TrackedDeviceAdded("PurpleHand_Right_001", vr::TrackedDeviceClass_Controller, m_pRightHand);

        // Инициализация UDP (порт 9001)
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        m_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        
        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(9001);
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        bind(m_udpSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr));

        u_long mode = 1;
        ioctlsocket(m_udpSocket, FIONBIO, &mode);

        return vr::VRInitError_None;
    }

    virtual void Cleanup() override {
        closesocket(m_udpSocket);
        WSACleanup();
        delete m_pLeftHand;
        delete m_pRightHand;
        m_pLeftHand = nullptr;
        m_pRightHand = nullptr;
    }

    virtual const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

    virtual void RunFrame() override {
        HandPacket packet;
        sockaddr_in clientAddr;
        int clientLength = sizeof(clientAddr);
        
        int bytesRead = recvfrom(m_udpSocket, (char*)&packet, sizeof(HandPacket), 0, (SOCKADDR*)&clientAddr, &clientLength);
        
        if (bytesRead == sizeof(HandPacket)) {
            if (packet.isRightHand == 1 && m_pRightHand) {
                m_pRightHand->UpdateSkeleton(packet);
            } else if (packet.isRightHand == 0 && m_pLeftHand) {
                m_pLeftHand->UpdateSkeleton(packet);
            }
        }
    }

    virtual bool ShouldBlockStandbyMode() override { return false; }
    virtual void EnterStandby() override {}
    virtual void LeaveStandby() override {}
};

CServerDriver_PurpleHands g_serverDriverPurpleHands;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#endif

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0) {
        return &g_serverDriverPurpleHands;
    }
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}