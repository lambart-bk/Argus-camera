/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/**
 * @file
 * <b>Libargus Extension: Sensor Private Metadata API</b>
 *
 * @b Description: This file defines the SensorPrivateMetadata extension.
 */

#ifndef _ARGUS_SENSOR_PRIVATE_METADATA_H
#define _ARGUS_SENSOR_PRIVATE_METADATA_H

namespace Argus
{
/**
 * Adds accessors for sensor embedded metadata. This data is metadata that the sensor embeds
 * inside the frame, the type and formating of which depends on the sensor. It is up to the
 * user to correctly parse the data based on the specifics of the sensor used.
 *
 *   - Ext::ISensorPrivateMetadataCaps: Determines whether a device is capable of
 *                                       private metadata output.
 *   - Ext::ISensorPrivateMetadataRequest: Enables private metadata output from a capture request.
 *   - Ext::ISensorPrivateMetadata: Accesses the sensor private metadata.
 *
 * @defgroup ArgusExtSensorPrivateMetadata Ext::SensorPrivateMetadata
 * @ingroup ArgusExtensions
 */
DEFINE_UUID(ExtensionName, EXT_SENSOR_PRIVATE_METADATA, 7acf4352,3a75,46e7,9af1,8d,71,da,83,15,23);

namespace Ext
{

/**
 * @class ISensorPrivateMetadataCaps
 *
 * Interface used to query the availability and size in bytes of sensor private metadata.
 *
 * @ingroup ArgusCameraDevice ArgusExtSensorPrivateMetadata
 */
DEFINE_UUID(InterfaceID, IID_SENSOR_PRIVATE_METADATA_CAPS, e492d2bf,5285,476e,94c5,ee,64,d5,3d,94,ef);
class ISensorPrivateMetadataCaps : public Interface
{
public:
    static const InterfaceID& id() { return IID_SENSOR_PRIVATE_METADATA_CAPS; }

    /**
     * Returns the size in bytes of the private metadata.
     */
    virtual size_t getMetadataSize() const = 0;

protected:
    ~ISensorPrivateMetadataCaps() {}
};

/**
 * @class ISensorPrivateMetadataRequest
 *
 * Interface used enable the output of sensor private metadata for a request.
 *
 * @ingroup ArgusRequest ArgusExtSensorPrivateMetadata
 */
DEFINE_UUID(InterfaceID, IID_SENSOR_PRIVATE_METADATA_REQUEST, 5c868b69,42f5,4ec9,9b93,44,11,c9,6c,02,e3);
class ISensorPrivateMetadataRequest : public Interface
{
public:
    static const InterfaceID& id() { return IID_SENSOR_PRIVATE_METADATA_REQUEST; }

    /**
     * Enables the sensor private metadata, will only work if the sensor supports embedded metadata.
     * @param[in] enable whether to output embedded metadata.
     */
    virtual void setMetadataEnable(bool enable) = 0;

    /**
     * Returns if the metadata is enabled for this request.
     */
    virtual bool getMetadataEnable() const = 0;

protected:
    ~ISensorPrivateMetadataRequest() {}
};

/**
 * @class ISensorPrivateMetadata
 *
 * Interface used to access sensor private metadata.
 *
 * @ingroup ArgusCaptureMetadata ArgusExtSensorPrivateMetadata
 */
DEFINE_UUID(InterfaceID, IID_SENSOR_PRIVATE_METADATA, 68cf6680,70d7,4b52,9a99,33,fb,65,81,a2,61);
class ISensorPrivateMetadata : public Interface
{
public:
    static const InterfaceID& id() { return IID_SENSOR_PRIVATE_METADATA; }

    /**
     * Returns the size of the embedded metadata.
     */
    virtual size_t getMetadataSize() const = 0;

    /**
     * Copies back the metadata to the provided memory location.
     * If the size of @a dst is smaller than the total size of the metadata, only the first
     * bytes up to size are copied.
     * @param [in,out] dst The pointer to the location where the data will be copied.
     *                     The caller is responsible for allocating and managing the memory.
     * @param [in] size The size of the destination.
     */
    virtual Status getMetadata(void *dst, size_t size) const = 0;

protected:
    ~ISensorPrivateMetadata() {}
};

} // namespace Ext

} // namespace Argus

#endif // _ARGUS_SENSOR_PRIVATE_METADATA_H
