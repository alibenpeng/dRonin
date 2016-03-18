/**
 ******************************************************************************
 *
 * @file       myfirstfc.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Alibenpeng Alibenpeng boards support Plugin
 * @{
 * @brief Plugin to support boards by Alibenpeng
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "myfirstfc.h"

#include <uavobjectmanager.h>
#include "uavobjectutil/uavobjectutilmanager.h"
#include <extensionsystem/pluginmanager.h>

#include "hwmyfirstfc.h"

/**
 * @brief MyFirstFC::MyFirstFC
 *  This is the MyFirstFC board definition
 */
MyFirstFC::MyFirstFC(void)
{
    // Initialize our USB Structure definition here:
    USBInfo board;
    board.vendorID = 0x16c0;
    board.productID = 0x05e1;

    setUSBInfo(board);

    boardType = 0x86;

    // Define the bank of channels that are connected to a given timer
    channelBanks.resize(6);
    channelBanks[0] = QVector<int> () << 1 << 2 << 3 << 4;
    channelBanks[1] = QVector<int> () << 5 << 6;
    channelBanks[2] = QVector<int> () << 7;
    channelBanks[3] = QVector<int> () << 8;
}

MyFirstFC::~MyFirstFC()
{

}

QString MyFirstFC::shortName()
{
    return QString("myfirstfc");
}

QString MyFirstFC::boardDescription()
{
    return QString("My first FC by Alexander Tuxen");
}

//! Return which capabilities this board has
bool MyFirstFC::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return false;
    case BOARD_CAPABILITIES_BAROS:
        return false;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    case BOARD_CAPABILITIES_OSD:
        return false;
    }
    return false;
}


/**
 * @brief MyFirstFC::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList MyFirstFC::getSupportedProtocols()
{

    return QStringList("uavtalk");
}

QPixmap MyFirstFC::getBoardPicture()
{
    return QPixmap(":/alibenpeng/images/myfirstfc.png");
}

QString MyFirstFC::getHwUAVO()
{
    return "HwMyFirstFC";
}

int MyFirstFC::queryMaxGyroRate()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwMyFirstFC *hwMyFirstFC = HwMyFirstFC::GetInstance(uavoManager);
    Q_ASSERT(hwMyFirstFC);
    if (!hwMyFirstFC)
        return 0;

    HwMyFirstFC::DataFields settings = hwMyFirstFC->getData();

    switch(settings.GyroRange) {
    case HwMyFirstFC::GYRORANGE_250:
        return 250;
    case HwMyFirstFC::GYRORANGE_500:
        return 500;
    case HwMyFirstFC::GYRORANGE_1000:
        return 1000;
    case HwMyFirstFC::GYRORANGE_2000:
        return 2000;
    default:
        return 500;
    }
}

QStringList MyFirstFC::getAdcNames()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwMyFirstFC *hwMyFirstFC = HwMyFirstFC::GetInstance(uavoManager);
    Q_ASSERT(hwMyFirstFC);
    if (!hwMyFirstFC)
        return QStringList();

    HwMyFirstFC::DataFields settings = hwMyFirstFC->getData();
    if (settings.RcvrPort == HwMyFirstFC::RCVRPORT_OUTPUTSADC ||
            settings.RcvrPort == HwMyFirstFC::RCVRPORT_PPMADC ||
            settings.RcvrPort == HwMyFirstFC::RCVRPORT_PPMOUTPUTSADC ||
            settings.RcvrPort == HwMyFirstFC::RCVRPORT_PPMPWMADC ||
            settings.RcvrPort == HwMyFirstFC::RCVRPORT_PWMADC) {
        return QStringList() << "IN 7" << "IN 8";
    }

    return QStringList() << "Disabled" << "Disabled";
}
