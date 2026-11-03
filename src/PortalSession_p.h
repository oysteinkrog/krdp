// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <type_traits>

#include <QDBusPendingCallWatcher>
#include <QElapsedTimer>
#include <QObject>
#include <QPoint>
#include <QPointer>

#include "krdp_export.h"

namespace KRdp
{

/**
 * Convenience class to help with making requests to the Desktop portal less cumbersome to work with.
 *
 * A request to the portal requires first calling a method on the portal, which
 * will return an object path that should be listened to for a signal. The
 * signal provides the actual proper data for the original method call.
 */
class PortalRequest : public QObject
{
    Q_OBJECT
public:
    /**
     * \param step Name of the step, for the log.
     * \param handleToken The handle_token in the call's options. The portal sends the Response
     * signal on a request path built from it, so this subscribes to that path before making
     * the call. Subscribing only after the call returned could miss a fast Response, and the
     * request then never finished: the client stayed on a black screen until KRDP's portal
     * timeout dropped it.
     * \param makeCall Makes the portal call.
     */
    template<typename ContextType, typename Callback>
    PortalRequest(const QString &step, const QString &handleToken, ContextType *context, Callback callback, const std::function<QDBusPendingCall()> &makeCall)
        : m_context(context)
        , m_step(step)
    {
        if constexpr (std::is_member_function_pointer<Callback>::value) {
            m_callback = std::bind(callback, context, std::placeholders::_1, std::placeholders::_2);
        } else {
            m_callback = callback;
        }

        m_elapsed.start();
        subscribe(requestPath(handleToken));
        auto watcher = new QDBusPendingCallWatcher(makeCall());
        watcher->waitForFinished();
        onStarted(watcher);
    }

private:
    static QString requestPath(const QString &handleToken);
    void subscribe(const QString &path);
    void unsubscribe();
    void onStarted(QDBusPendingCallWatcher *watcher);
    Q_SLOT void onFinished(uint code, const QVariantMap &result);

    QPointer<QObject> m_context;
    std::function<void(uint, const QVariantMap &)> m_callback;
    QString m_step;
    QString m_path;
    QElapsedTimer m_elapsed;
};

struct PortalSessionStream {
    uint nodeId;
    QVariantMap map;
};

}

Q_DECLARE_METATYPE(KRdp::PortalSessionStream)
