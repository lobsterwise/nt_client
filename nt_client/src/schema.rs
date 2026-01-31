//! Schema management for `struct`s and/or `protobuf`s.

use std::{collections::HashMap, convert::Infallible, sync::Arc};

#[cfg(feature = "protobuf")]
use protobuf::{MessageFull, descriptor::FileDescriptorProto, reflect::FileDescriptor};
use tokio::sync::{Mutex, broadcast};
use tracing::{debug, warn};

#[cfg(feature = "protobuf")]
use crate::protobuf::ProtobufData;

#[cfg(feature = "struct")]
use crate::{r#struct::{StructData, StructSchema, parse::{ParsedStruct, parse_schema}}};

use crate::{ClientHandle, data::{DataType, NetworkTableData}, error::ConnectionClosedError, publish::NewPublisherError, subscribe::{ReceivedMessage, SubscriptionOptions}, topic::Properties};

/// The name of a schema, `/.schema/{name}`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum SchemaName {
    /// A struct schema, `struct:{name}`.
    #[cfg(feature = "struct")]
    Struct(String),

    /// A protobuf schema, `proto:{name}`.
    #[cfg(feature = "protobuf")]
    Proto(String),
}

/// A parsed schema.
#[derive(Debug, Clone, PartialEq)]
pub enum Schema {
    /// Schema for a struct.
    #[cfg(feature = "struct")]
    Struct(ParsedStruct),

    /// Schema for a protobuf.
    ///
    /// `Box`ed to reduce enum size.
    #[cfg(feature = "protobuf")]
    Proto(Box<FileDescriptorProto>),
}

/// A clonable schema manager.
///
/// Clones will share the same internal schema map.
/// ```
#[derive(Debug, Clone)]
pub struct SchemaManager {
    schemas: Arc<Mutex<HashMap<SchemaName, Schema>>>,
    handle: ClientHandle,
}

impl SchemaManager {
    pub(crate) fn new(schemas: Arc<Mutex<HashMap<SchemaName, Schema>>>, handle: ClientHandle) -> Self {
        Self {
            schemas,
            handle,
        }
    }

    /// Publishes the struct schema for `T`, as well as any nested structs that `T` references.
    ///
    /// The topic the server is publishing to is `retained` by default.
    ///
    /// Nothing is published if a schema for `T` has already been parsed.
    #[cfg(feature = "struct")]
    pub async fn publish_struct<T: StructData>(&mut self) -> Result<(), PublishSchemaError> {
        {
            let schemas = self.schemas.lock().await;
            if schemas.contains_key(&SchemaName::Struct(T::struct_type_name())) {
                return Ok(());
            }
        }

        // TODO: publish nested structs

        let publisher = self.handle.struct_schema_topic::<T>().publish::<StructSchema>(Properties { retained: Some(true), ..Default::default() }).await?;
        publisher.set_default(T::schema()).await?;
        Ok(())
    }

    /// Publishes the protobuf schema for `T`, as well as any nested protobufs `T` depends on.
    ///
    /// The topic the server is publishing to is `retained` by default.
    ///
    /// Nothing is published if a schema for `T` has already been parsed.
    #[cfg(feature = "protobuf")]
    pub async fn publish_proto<T: ProtobufData>(&mut self) -> Result<(), PublishSchemaError> {
        self.publish_file_descriptor(T::message_descriptor().file_descriptor()).await
    }

    #[cfg(feature = "protobuf")]
    async fn publish_file_descriptor(&mut self, descriptor: &FileDescriptor) -> Result<(), PublishSchemaError> {
        {
            let schemas = self.schemas.lock().await;
            if schemas.contains_key(&SchemaName::Proto(descriptor.name().to_owned())) {
                return Ok(());
            }
        }

        for dep in descriptor.deps() {
            Box::pin(self.publish_file_descriptor(dep)).await?;
        }

        let publisher = self.handle.protobuf_schema_topic(descriptor).publish::<FileDescriptorProto>(Properties { retained: Some(true), ..Default::default() }).await?;
        publisher.set_default(descriptor.proto().clone()).await?;
        // publisher is retained, so we can drop it here
        Ok(())
    }

    /// Watches the `/.schema/` topic, parsing and adding any schemas it encounters to its shared
    /// schema map.
    ///
    /// Specific schemas types will only be parsed from enabled features, e.g. if only the `struct`
    /// feature is enabled only `struct` schemas will be recognized and parsed.
    ///
    /// Note that this method will only return if it encounters an error (notice the [`Infallible`]
    /// `Ok` type).
    pub async fn watch(self) -> Result<Infallible, broadcast::error::RecvError> {
        let mut sub = self.handle.schema_topic().subscribe(SubscriptionOptions { prefix: Some(true), ..Default::default() }).await.map_err(|_| broadcast::error::RecvError::Closed)?;
        loop {
            if let ReceivedMessage::Updated((topic, value)) = sub.recv().await? {
                let type_name = topic.name().strip_prefix("/.schema/").expect("/.schema/ prefix");

                match topic.r#type() {
                    #[cfg(feature = "struct")]
                    DataType::StructSchema => {
                        match type_name.strip_prefix("struct:") {
                            Some(type_name) => {
                                match StructSchema::from_value(value).and_then(|schema| parse_schema(&schema.0).ok()) {
                                    Some(schema) => {
                                        debug!("[schema {type_name}] parsed as {schema:?}");
                                        let mut schemas = self.schemas.lock().await;
                                        schemas.insert(SchemaName::Struct(type_name.to_owned()), Schema::Struct(schema));
                                    },
                                    None => warn!("[schema {type_name}] invalid struct schema"),
                                }
                            },
                            None => warn!("[schema {type_name}] expected struct schema to start with `struct:`"),
                        }
                    },
                    #[cfg(feature = "protobuf")]
                    DataType::Protobuf(proto) if proto == FileDescriptorProto::descriptor().name() => {
                        match type_name.strip_prefix("proto:") {
                            Some(type_name) => {
                                match FileDescriptorProto::from_value(value) {
                                    Some(schema) => {
                                        debug!("[schema {type_name}] parsed as {schema:?}");
                                        let mut schemas = self.schemas.lock().await;
                                        schemas.insert(SchemaName::Proto(type_name.to_owned()), Schema::Proto(schema.into()));
                                    },
                                    None => warn!("[schema {type_name}] invalid protobuf schema"),
                                }
                            },
                            None => warn!("[schema {type_name}] expected protobuf schema to start with `proto:`"),
                        }
                    },
                    r#type => warn!("[schema {type_name}] invalid schema type {type:?}"),
                }
            }
        }
    }
}

/// Errors that can occur when publishing a schema.
#[derive(thiserror::Error, Debug, Clone, PartialEq, Eq)]
pub enum PublishSchemaError {
    /// An error occurred when creating a new publisher.
    #[error(transparent)]
    NewPublisher(#[from] NewPublisherError),

    /// The `NetworkTables` connection was closed.
    #[error(transparent)]
    ConnectionClosed(#[from] ConnectionClosedError),
}

